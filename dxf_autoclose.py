#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DXF Contour Fixer v4
- Очистка (isolated / open chains / zero-length / min length)
- Редактирование (select / delete / undo / isolate)
- Измерения (P-P, L-L) в мм с привязкой
- Слияние в замкнутые LWPOLYLINE и экспорт DXF / PLT для EZCAD
"""

import os, sys, io, tkinter as tk
from tkinter import filedialog, messagebox, ttk
from math import pi, cos, sin, radians, hypot, floor

try:
    import ezdxf
    from ezdxf.math import Vec2, bulge_to_arc
except ImportError:
    _r = tk.Tk(); _r.withdraw()
    messagebox.showerror("Missing ezdxf", "pip install ezdxf")
    sys.exit(1)


UNIT_TO_MM = {0: 1.0, 1: 25.4, 2: 304.8, 3: 1609344.0, 4: 1.0, 5: 10.0,
              6: 1000.0, 7: 1000000.0, 8: 0.0000254, 9: 0.0254, 10: 914.4}


# ============ geometry helpers ============

def point_to_seg(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx-ax, by-ay
    L2 = dx*dx + dy*dy
    if L2 < 1e-20:
        return hypot(px-ax, py-ay), (ax, ay)
    t = ((px-ax)*dx + (py-ay)*dy) / L2
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t*dx, ay + t*dy
    return hypot(px-cx, py-cy), (cx, cy)


def seg_seg_dist_pts(a1, a2, b1, b2):
    x1, y1 = a1; x2, y2 = a2; x3, y3 = b1; x4, y4 = b2
    den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    if abs(den) > 1e-20:
        t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / den
        u = -((x1-x2)*(y1-y3) - (y1-y2)*(x1-x3)) / den
        if 0 <= t <= 1 and 0 <= u <= 1:
            ix = x1 + t*(x2-x1); iy = y1 + t*(y2-y1)
            return 0.0, (ix, iy), (ix, iy)
    best = (float('inf'), None, None)
    for p, sa, sb in [(a1, b1, b2), (a2, b1, b2), (b1, a1, a2), (b2, a1, a2)]:
        d, cp = point_to_seg(p, sa, sb)
        if d < best[0]:
            best = (d, p, cp) if p in (a1, a2) else (d, cp, p)
    return best


# ============ entity -> polyline ============

def entity_full_path(e):
    t = e.dxftype()
    try:
        if t == 'LINE':
            return [Vec2(e.dxf.start.x, e.dxf.start.y),
                    Vec2(e.dxf.end.x, e.dxf.end.y)]
        if t == 'ARC':
            cx, cy, r = e.dxf.center.x, e.dxf.center.y, e.dxf.radius
            a1, a2 = radians(e.dxf.start_angle), radians(e.dxf.end_angle)
            if a2 <= a1: a2 += 2*pi
            n = max(12, int((a2-a1) / radians(5)))
            return [Vec2(cx + r*cos(a1 + (a2-a1)*i/n),
                         cy + r*sin(a1 + (a2-a1)*i/n)) for i in range(n+1)]
        if t == 'CIRCLE':
            cx, cy, r = e.dxf.center.x, e.dxf.center.y, e.dxf.radius
            n = 96
            pts = [Vec2(cx + r*cos(2*pi*i/n), cy + r*sin(2*pi*i/n)) for i in range(n)]
            pts.append(pts[0])
            return pts
        if t == 'LWPOLYLINE':
            raw = list(e.get_points('xyb'))
            if len(raw) < 2: return None
            pts = []
            for i, (x, y, b) in enumerate(raw):
                pts.append(Vec2(x, y))
                if b != 0 and i < len(raw) - 1:
                    nxt = raw[i+1]
                    try:
                        c, sa, ea, rr = bulge_to_arc(Vec2(x, y), Vec2(nxt[0], nxt[1]), b)
                    except Exception:
                        continue
                    if b > 0:
                        if ea <= sa: ea += 2*pi
                    else:
                        if ea >= sa: ea -= 2*pi
                    n = max(4, int(abs(ea-sa) / radians(10)))
                    for k in range(1, n):
                        a = sa + (ea-sa)*k/n
                        pts.append(Vec2(c.x + rr*cos(a), c.y + rr*sin(a)))
            if e.closed and (pts[0]-pts[-1]).magnitude > 1e-12:
                pts.append(pts[0])
            return pts
        if t == 'POLYLINE':
            pts = [Vec2(v.dxf.location.x, v.dxf.location.y) for v in e.vertices]
            if e.is_closed and pts and (pts[0]-pts[-1]).magnitude > 1e-12:
                pts.append(pts[0])
            return pts if len(pts) >= 2 else None
        if t == 'SPLINE':
            pts = [Vec2(p.x, p.y) for p in e.flattening(0.1)]
            return pts if len(pts) >= 2 else None
    except Exception:
        return None
    return None


def entity_is_closed(e):
    t = e.dxftype()
    if t == 'CIRCLE': return True
    if t == 'LWPOLYLINE': return e.closed
    if t == 'POLYLINE': return e.is_closed
    try:
        if t == 'SPLINE': return e.closed
    except Exception:
        pass
    return False


def polyline_length(pl):
    return sum((pl[i+1]-pl[i]).magnitude for i in range(len(pl)-1))


def polyline_area(pl):
    s = 0.0
    for i in range(len(pl)-1):
        s += pl[i].x * pl[i+1].y - pl[i+1].x * pl[i].y
    return abs(s) * 0.5


# ============ connectivity + cleanup ============

def build_graph(doc, tol):
    msp = doc.modelspace()
    items, nodes, buckets = [], [], {}
    cell = max(tol*2, 1e-9)
    def get_node(p):
        kx, ky = floor(p.x/cell), floor(p.y/cell)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for i in buckets.get((kx+dx, ky+dy), ()):
                    if (nodes[i]-p).magnitude < tol:
                        return i
        nodes.append(p)
        buckets.setdefault((kx, ky), []).append(len(nodes)-1)
        return len(nodes)-1
    for e in msp:
        pl = entity_full_path(e)
        if not pl or len(pl) < 2: continue
        if entity_is_closed(e):
            items.append({'entity': e, 'polyline': pl,
                          'start': -1, 'end': -1, 'is_closed': True})
            continue
        a = get_node(pl[0]); b = get_node(pl[-1])
        items.append({'entity': e, 'polyline': pl,
                      'start': a, 'end': b, 'is_closed': False})
    adj = {}
    for i, it in enumerate(items):
        if it['is_closed']: continue
        adj.setdefault(it['start'], []).append(i)
        adj.setdefault(it['end'], []).append(i)
    return items, nodes, adj


def clean_doc(doc, tol, opts, log):
    items, nodes, adj = build_graph(doc, tol)
    remove = set()

    if opts.get('zero_length'):
        for i, it in enumerate(items):
            if polyline_length(it['polyline']) < tol:
                remove.add(i)

    min_len = opts.get('min_length', 0.0)
    if min_len > 0:
        for i, it in enumerate(items):
            if polyline_length(it['polyline']) < min_len:
                remove.add(i)

    if opts.get('isolated'):
        for i, it in enumerate(items):
            if it['is_closed']: continue
            a, b = it['start'], it['end']
            if a == b: continue
            deg_a = sum(1 for j in adj.get(a, []) if j != i)
            deg_b = sum(1 for j in adj.get(b, []) if j != i)
            if deg_a == 0 and deg_b == 0:
                remove.add(i)

    if opts.get('open_chains'):
        parent = {}
        def find(x):
            parent.setdefault(x, x)
            while parent[x] != x:
                parent[x] = parent[parent[x]]; x = parent[x]
            return x
        def union(a, b):
            ra, rb = find(a), find(b)
            if ra != rb: parent[ra] = rb
        for ni, eis in adj.items():
            for k in range(1, len(eis)): union(eis[0], eis[k])
        groups = {}
        for i in range(len(items)):
            if items[i]['is_closed']: continue
            groups.setdefault(find(i), []).append(i)
        for g_root, eis in groups.items():
            node_set = set()
            for i in eis:
                node_set.add(items[i]['start']); node_set.add(items[i]['end'])
            has_open = any(len(adj.get(n, [])) == 1 for n in node_set)
            if has_open:
                for i in eis: remove.add(i)

    msp = doc.modelspace()
    for i in remove:
        try: msp.delete_entity(items[i]['entity'])
        except Exception: pass
    return len(remove)


# ============ merge ============

def merge_to_contours(doc, tol, log):
    items, nodes, adj = build_graph(doc, tol)
    preserved = [it['polyline'] for it in items if it['is_closed']]
    open_items = [it for it in items if not it['is_closed']]
    used = [False]*len(open_items)
    oadj = {}
    for i, it in enumerate(open_items):
        oadj.setdefault(it['start'], []).append(i)
        oadj.setdefault(it['end'], []).append(i)
    log(f"  Открытых сегментов: {len(open_items)} | уже замкнутых: {len(preserved)}")
    parent = list(range(len(nodes)))
    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]; x = parent[x]
        return x
    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb: parent[ra] = rb
    for it in open_items: union(it['start'], it['end'])
    comps = {}
    for n_idx in oadj: comps.setdefault(find(n_idx), []).append(n_idx)
    traced = []
    for root, members in comps.items():
        start = None
        for n_idx in members:
            if sum(1 for j in oadj.get(n_idx, []) if not used[j]) == 1:
                start = n_idx; break
        if start is None: start = members[0]
        while True:
            avail = [j for j in oadj.get(start, []) if not used[j]]
            if not avail:
                found = None
                for n_idx in members:
                    for j in oadj.get(n_idx, []):
                        if not used[j]:
                            found = n_idx; break
                    if found is not None: break
                if found is None: break
                start = found
            path = [nodes[start]]; cur = start
            while True:
                av = [j for j in oadj.get(cur, []) if not used[j]]
                if not av: break
                if len(path) >= 2:
                    prev, last = path[-2], path[-1]
                    def rank(j):
                        it = open_items[j]
                        other = it['end'] if it['start'] == cur else it['start']
                        tgt = nodes[other]
                        v1x, v1y = last.x-prev.x, last.y-prev.y
                        v2x, v2y = tgt.x-last.x, tgt.y-last.y
                        m = hypot(v1x, v1y)*hypot(v2x, v2y)
                        if m < 1e-12: return 1.0
                        return -(v1x*v2x + v1y*v2y)/m
                    av.sort(key=rank)
                j = av[0]; used[j] = True
                it = open_items[j]; pl = it['polyline']
                if it['start'] == cur:
                    path.extend(pl[1:]); cur = it['end']
                else:
                    path.extend(list(reversed(pl))[1:]); cur = it['start']
            if (path[-1]-path[0]).magnitude > tol:
                path.append(path[0])
                log(f"  Замкнут контур: {len(path)} точек")
            else:
                path[-1] = path[0]
            if len(path) >= 3: traced.append(path)
    return preserved + traced


# ============ writers ============

def write_clean_dxf(contours, out_path, all_ccw=False, min_area_doc2=0.0):
    doc = ezdxf.new('R2010')
    doc.header['$INSUNITS'] = 4  # mm
    msp = doc.modelspace()
    doc.layers.add("CONTOUR", color=7)
    n, skipped = 0, 0
    for pts in contours:
        if len(pts) < 3: continue
        if min_area_doc2 > 0 and polyline_area(pts) < min_area_doc2:
            skipped += 1; continue
        pts_use = list(pts)
        if all_ccw and polyline_area(pts_use) < 0:
            pts_use = list(reversed(pts_use))
        if (pts_use[0]-pts_use[-1]).magnitude < 1e-9:
            coords = [(p.x, p.y) for p in pts_use[:-1]]
        else:
            coords = [(p.x, p.y) for p in pts_use]
        msp.add_lwpolyline(coords, close=True, dxfattribs={'layer': 'CONTOUR'})
        n += 1
    doc.saveas(out_path)
    return n, skipped


def write_plt(contours, out_path, units_per_mm=40.0, margin_mm=1.0, min_area_doc2=0.0):
    filtered = []
    skipped = 0
    for pts in contours:
        if len(pts) < 3: continue
        if min_area_doc2 > 0 and polyline_area(pts) < min_area_doc2:
            skipped += 1; continue
        filtered.append(pts)
    if not filtered:
        raise ValueError("Нет контуров после фильтрации")
    xs = [p.x for pts in filtered for p in pts]
    ys = [p.y for pts in filtered for p in pts]
    minx, miny = min(xs), min(ys)
    margin = margin_mm * units_per_mm
    def hp(p):
        return (int(round((p.x-minx)*units_per_mm + margin)),
                int(round((p.y-miny)*units_per_mm + margin)))
    lines = ["IN;", "PA;", "SP1;"]
    n = 0
    for pts in filtered:
        if (pts[0]-pts[-1]).magnitude > 1e-9:
            pts = list(pts) + [pts[0]]
        h = [hp(p) for p in pts]
        x0, y0 = h[0]
        lines.append(f"PU{x0},{y0};")
        CH = 20
        for i in range(1, len(h), CH):
            lines.append("PD" + ",".join(f"{x},{y}" for x, y in h[i:i+CH]) + ";")
        lines.append("PU;"); n += 1
    lines.append("PG;")
    with open(out_path, 'w', newline='\r\n') as f:
        f.write("\n".join(lines))
    return n, skipped


# ============ GUI ============

class DxfApp:
    def __init__(self, root):
        self.root = root
        root.title("DXF Contour Fixer v4")
        root.geometry("1300x850")

        self.doc = None
        self.file_path = None
        self.entity_cache = []
        self.contours = None
        self.break_points = []
        self.bbox = None
        self.scale = 1.0
        self.ox = 0.0; self.oy = 0.0
        self.unit_to_mm = 1.0
        self.unit_name = 'мм'

        self.selection = set()
        self.measure_p1 = None
        self.measure_e1 = None
        self.measure_result = None  # (p1, p2, d_doc, dx_doc, dy_doc)

        self.undo_stack = []
        self._pan = None
        self._box_start = None
        self._box_end = None

        self._build_ui()

    # ---------- UI ----------
    def _build_ui(self):
        r1 = tk.Frame(self.root, bd=1, relief='raised'); r1.pack(side='top', fill='x')
        tk.Button(r1, text="Открыть DXF", command=self.open_file).pack(side='left', padx=2, pady=2)
        tk.Button(r1, text="Найти разрывы", command=self.find_breaks).pack(side='left', padx=2)
        tk.Button(r1, text="Слить в контуры", command=self.do_merge).pack(side='left', padx=2)
        tk.Button(r1, text="Сохранить DXF", command=self.save_dxf).pack(side='left', padx=2)
        tk.Button(r1, text="Сохранить PLT", command=self.save_plt).pack(side='left', padx=2)
        tk.Button(r1, text="Вписать", command=self.fit_view).pack(side='left', padx=8)
        tk.Label(r1, text="Допуск (мм):").pack(side='left', padx=(10, 2))
        self.tol_var = tk.StringVar(value="0.05")
        tk.Entry(r1, textvariable=self.tol_var, width=6).pack(side='left')
        tk.Label(r1, text="Мин.площадь (мм²):").pack(side='left', padx=(8, 2))
        self.min_area_var = tk.StringVar(value="0")
        tk.Entry(r1, textvariable=self.min_area_var, width=8).pack(side='left')
        self.ccw_var = tk.BooleanVar(value=False)
        tk.Checkbutton(r1, text="Все CCW", variable=self.ccw_var).pack(side='left', padx=4)
        tk.Label(r1, text="Ед.:").pack(side='left', padx=(10, 2))
        self.unit_var = tk.StringVar(value='мм')
        cb = ttk.Combobox(r1, textvariable=self.unit_var,
                          values=['мм', 'см', 'дюйм'], width=6, state='readonly')
        cb.pack(side='left')
        cb.bind('<<ComboboxSelected>>', lambda e: self._on_unit_change())

        r2 = tk.Frame(self.root, bd=1, relief='raised'); r2.pack(side='top', fill='x')
        tk.Label(r2, text="Режим:").pack(side='left', padx=(4, 2))
        self.mode_var = tk.StringVar(value='select')
        for val, txt in [('select', 'Выбор'),
                         ('measure_pp', 'Измерить P-P'),
                         ('measure_ll', 'Измерить L-L')]:
            tk.Radiobutton(r2, text=txt, variable=self.mode_var, value=val,
                           command=self.set_mode).pack(side='left', padx=2)
        tk.Button(r2, text="Удалить выбранное", command=self.delete_selection).pack(side='left', padx=6)
        tk.Button(r2, text="Только выбранное", command=self.isolate_selection).pack(side='left', padx=2)
        tk.Button(r2, text="Очистить мусор…", command=self.cleanup_dialog).pack(side='left', padx=2)
        tk.Button(r2, text="Отменить", command=self.undo).pack(side='left', padx=2)
        tk.Button(r2, text="Сбросить измерение", command=self.clear_measure).pack(side='left', padx=6)
        tk.Label(r2, text="ПКМ / СКМ — панорама, колесо — зум",
                 fg='gray').pack(side='right', padx=8)

        paned = tk.PanedWindow(self.root, orient='vertical', sashwidth=6)
        paned.pack(fill='both', expand=True)
        cf = tk.Frame(paned)
        self.canvas = tk.Canvas(cf, bg='white', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        paned.add(cf, stretch='always')
        lf = tk.Frame(paned)
        tk.Label(lf, text="Лог:", anchor='w').pack(fill='x')
        self.log_text = tk.Text(lf, height=7, font=('Courier', 9))
        self.log_text.pack(fill='both', expand=True)
        paned.add(lf)

        sb = tk.Frame(self.root, bd=1, relief='sunken'); sb.pack(side='bottom', fill='x')
        self.measure_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="Откройте DXF.")
        self.coord_var = tk.StringVar(value="")
        tk.Label(sb, textvariable=self.measure_var, anchor='w',
                 font=('Courier', 9, 'bold'), fg='#0033cc').pack(side='left', padx=4)
        tk.Label(sb, textvariable=self.status_var, anchor='w').pack(side='left', fill='x',
                                                                  expand=True, padx=8)
        tk.Label(sb, textvariable=self.coord_var, anchor='e',
                 font=('Courier', 9)).pack(side='right', padx=4)

        self.canvas.bind('<Configure>', lambda e: self.redraw())
        self.canvas.bind('<MouseWheel>', self._wheel)
        self.canvas.bind('<Button-4>', self._wheel)
        self.canvas.bind('<Button-5>', self._wheel)
        self.canvas.bind('<ButtonPress-1>', self._on_lpress)
        self.canvas.bind('<B1-Motion>', self._on_ldrag)
        self.canvas.bind('<ButtonRelease-1>', self._on_lrelease)
        self.canvas.bind('<Motion>', self._on_motion)
        self.canvas.bind('<ButtonPress-2>', self._pan_start)
        self.canvas.bind('<B2-Motion>', self._pan_move)
        self.canvas.bind('<ButtonRelease-2>', self._pan_end)
        self.canvas.bind('<ButtonPress-3>', self._pan_start)
        self.canvas.bind('<B3-Motion>', self._pan_move)
        self.canvas.bind('<ButtonRelease-3>', self._pan_end)
        self.root.bind('<Delete>', lambda e: self.delete_selection())
        self.root.bind('<Escape>', lambda e: self.clear_measure_or_sel())
        self.root.bind('<Control-z>', lambda e: self.undo())
        self.root.bind('<Control-a>', lambda e: self.select_all())

    # ---------- helpers ----------
    def log(self, m):
        self.log_text.insert('end', m+'\n'); self.log_text.see('end'); self.root.update_idletasks()

    def status(self, m):
        self.status_var.set(m); self.root.update_idletasks()

    def set_mode(self):
        self.measure_p1 = None; self.measure_e1 = None
        self.status(f"Режим: {self.mode_var.get()}")

    def _on_unit_change(self):
        u = self.unit_var.get()
        if u == 'дюйм': self.unit_to_mm = 25.4; self.unit_name = 'дюйм'
        elif u == 'см': self.unit_to_mm = 10.0; self.unit_name = 'см'
        else: self.unit_to_mm = 1.0; self.unit_name = 'мм'
        self.status(f"Единицы DXF: {self.unit_name}  (1 ед. = {self.unit_to_mm} мм)")
        self.redraw()

    def _detect_units(self):
        ins = 0
        try:
            ins = int(self.doc.header.get('$INSUNITS', 0) or 0)
        except Exception:
            ins = 0
        if ins == 1: self.unit_var.set('дюйм')
        elif ins == 5: self.unit_var.set('см')
        else: self.unit_var.set('мм')
        self._on_unit_change()

    def get_tol_doc(self):
        try:
            mm = float(self.tol_var.get())
            if mm <= 0: raise ValueError
        except ValueError:
            messagebox.showerror("Ошибка", "Допуск — положительное число (мм)")
            return None
        return mm / self.unit_to_mm if self.unit_to_mm > 0 else mm

    def get_min_area_doc2(self):
        try:
            mm2 = float(self.min_area_var.get() or "0")
        except ValueError:
            return 0.0
        return mm2 / (self.unit_to_mm ** 2) if self.unit_to_mm > 0 else 0.0

    def s2w(self, sx, sy):
        ch = self.canvas.winfo_height()
        return sx/self.scale + self.ox, (ch - sy)/self.scale + self.oy

    def w2s(self, wx, wy):
        ch = self.canvas.winfo_height()
        return (wx - self.ox)*self.scale, ch - (wy - self.oy)*self.scale

    # ---------- cache ----------
    def _rebuild_cache(self):
        self.entity_cache = []
        if not self.doc: return
        for e in self.doc.modelspace():
            pl = entity_full_path(e)
            if not pl or len(pl) < 2: continue
            xs = [p.x for p in pl]; ys = [p.y for p in pl]
            self.entity_cache.append({
                'entity': e,
                'handle': e.dxf.handle,
                'polyline': pl,
                'bbox': (min(xs), min(ys), max(xs), max(ys)),
            })
        x0 = y0 = float('inf'); x1 = y1 = float('-inf')
        for it in self.entity_cache:
            bx0, by0, bx1, by1 = it['bbox']
            x0 = min(x0, bx0); y0 = min(y0, by0)
            x1 = max(x1, bx1); y1 = max(y1, by1)
        self.bbox = (x0, y0, x1, y1) if x0 < float('inf') else None

    def _find_item(self, h):
        for it in self.entity_cache:
            if it['handle'] == h: return it
        return None

    # ---------- open ----------
    def open_file(self):
        p = filedialog.askopenfilename(filetypes=[("DXF", "*.dxf"), ("All", "*.*")])
        if not p: return
        try:
            self.status("Загрузка…")
            self.doc = ezdxf.readfile(p)
            self.file_path = p
            self.contours = None
            self.break_points = []
            self.selection.clear()
            self.measure_p1 = None; self.measure_e1 = None; self.measure_result = None
            self.undo_stack.clear()
            self.log_text.delete('1.0', 'end')
            self.log(f"Загружено: {p}")
            self._detect_units()
            self._rebuild_cache()
            self.fit_view()
            self.status(f"Загружено: {len(self.entity_cache)} объектов.")
        except Exception as ex:
            messagebox.showerror("Ошибка", str(ex)); self.status("Ошибка")

    # ---------- find breaks ----------
    def find_breaks(self):
        if not self.doc:
            messagebox.showwarning("Нет файла", "Откройте DXF"); return
        tol = self.get_tol_doc()
        if tol is None: return
        pts = []
        for it in self.entity_cache:
            if it['entity'].dxftype() == 'CIRCLE': continue
            if entity_is_closed(it['entity']): continue
            pl = it['polyline']
            if (pl[0]-pl[-1]).magnitude < tol: continue
            pts.append(pl[0]); pts.append(pl[-1])
        if not pts:
            self.break_points = []
        else:
            n = len(pts); parent = list(range(n))
            def find(x):
                while parent[x] != x:
                    parent[x] = parent[parent[x]]; x = parent[x]
                return x
            def union(a, b):
                ra, rb = find(a), find(b)
                if ra != rb: parent[ra] = rb
            cell = max(tol*2, 1e-9); b = {}
            for i, p in enumerate(pts):
                b.setdefault((floor(p.x/cell), floor(p.y/cell)), []).append(i)
            for i, p in enumerate(pts):
                kx, ky = floor(p.x/cell), floor(p.y/cell)
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for j in b.get((kx+dx, ky+dy), ()):
                            if j <= i: continue
                            if (pts[j]-p).magnitude <= tol: union(i, j)
            groups = {}
            for i in range(n): groups.setdefault(find(i), []).append(i)
            self.break_points = [(pts[g[0]].x, pts[g[0]].y)
                                  for g in groups.values() if len(g) == 1]
        self.log(f"Разрывов: {len(self.break_points)}")
        self.status(f"Красных точек (разрывов): {len(self.break_points)}")
        self.redraw()

    # ---------- merge ----------
    def do_merge(self):
        if not self.doc:
            messagebox.showwarning("Нет файла", "Откройте DXF"); return
        tol = self.get_tol_doc()
        if tol is None: return
        try:
            self.log(f"Слияние (допуск {tol:.4f} ед.)…")
            self.contours = merge_to_contours(self.doc, tol, self.log)
            self.log(f"Итого контуров: {len(self.contours)}")
            self.break_points = []
            self.selection.clear()
            self.fit_view()
            self.status(f"Слито контуров: {len(self.contours)}. Сохраните DXF/PLT.")
        except Exception as ex:
            messagebox.showerror("Ошибка", str(ex)); self.log(f"ОШИБКА: {ex}")

    # ---------- save ----------
    def save_dxf(self):
        if not self.doc:
            messagebox.showwarning("Нет файла", "Откройте DXF"); return
        default = os.path.splitext(os.path.basename(self.file_path or "output"))[0]
        default += "_clean.dxf" if self.contours else "_edited.dxf"
        out = filedialog.asksaveasfilename(defaultextension=".dxf",
                                            filetypes=[("DXF", "*.dxf")],
                                            initialfile=default)
        if not out: return
        try:
            if self.contours:
                ma = self.get_min_area_doc2()
                n, sk = write_clean_dxf(self.contours, out,
                                         all_ccw=self.ccw_var.get(), min_area_doc2=ma)
                self.log(f"DXF: {n} LWPOLYLINE, отфильтровано: {sk}")
                msg = f"Сохранено {n} контуров (отфильтровано {sk})."
            else:
                self.doc.saveas(out)
                self.log(f"DXF сохранён как есть: {out}")
                msg = "DXF сохранён (без слияния)."
            messagebox.showinfo("Готово", msg + f"\n{out}")
            self.status("Сохранено.")
        except Exception as ex:
            messagebox.showerror("Ошибка", str(ex))

    def save_plt(self):
        if not self.contours:
            messagebox.showwarning("Нет контуров", "Сначала «Слить в контуры»"); return
        default = os.path.splitext(os.path.basename(self.file_path or "output"))[0] + ".plt"
        out = filedialog.asksaveasfilename(defaultextension=".plt",
                                            filetypes=[("PLT/HPGL", "*.plt"), ("HPGL", "*.hpgl")],
                                            initialfile=default)
        if not out: return
        try:
            ma = self.get_min_area_doc2()
            # convert PLT scale: 40 units/mm
            n, sk = write_plt(self.contours, out, min_area_doc2=ma)
            self.log(f"PLT: {n} контуров, отфильтровано: {sk}")
            messagebox.showinfo("Готово", f"PLT сохранён: {n} контуров\n{out}")
            self.status(f"PLT: {n} контуров.")
        except Exception as ex:
            messagebox.showerror("Ошибка", str(ex))

    # ---------- undo ----------
    def push_undo(self):
        if not self.doc: return
        try:
            buf = io.StringIO()
            self.doc.write(buf)
            self.undo_stack.append(buf.getvalue())
            if len(self.undo_stack) > 20: self.undo_stack.pop(0)
        except Exception as ex:
            self.log(f"Undo push failed: {ex}")

    def undo(self):
        if not self.undo_stack:
            self.status("Нечего отменять."); return
        data = self.undo_stack.pop()
        try:
            self.doc = ezdxf.read(io.StringIO(data))
            self.selection.clear()
            self.contours = None
            self.break_points = []
            self.measure_result = None; self.measure_p1 = None; self.measure_e1 = None
            self._rebuild_cache()
            self.fit_view()
            self.log("Отменено.")
        except Exception as ex:
            self.log(f"Undo failed: {ex}")

    # ---------- selection ----------
    def select_all(self):
        self.selection = {it['handle'] for it in self.entity_cache}
        self.redraw()

    def delete_selection(self):
        if not self.selection or not self.doc: return
        self.push_undo()
        msp = self.doc.modelspace()
        n = 0
        for it in list(self.entity_cache):
            if it['handle'] in self.selection:
                try:
                    msp.delete_entity(it['entity']); n += 1
                except Exception: pass
        self.selection.clear()
        self.contours = None
        self.break_points = []
        self._rebuild_cache()
        self.log(f"Удалено: {n}")
        self.status(f"Удалено: {n}")
        self.redraw()

    def isolate_selection(self):
        if not self.selection or not self.doc: return
        self.push_undo()
        msp = self.doc.modelspace()
        keep = set(self.selection)
        n = 0
        for it in list(self.entity_cache):
            if it['handle'] not in keep:
                try:
                    msp.delete_entity(it['entity']); n += 1
                except Exception: pass
        self.selection.clear()
        self.contours = None
        self.break_points = []
        self._rebuild_cache()
        self.fit_view()
        self.log(f"Изолировано. Удалено: {n}")
        self.status(f"Оставлено: {len(self.entity_cache)}")

    def clear_measure_or_sel(self):
        if self.measure_p1 or self.measure_e1 or self.measure_result:
            self.clear_measure()
        else:
            self.selection.clear()
            self.redraw()

    def clear_measure(self):
        self.measure_p1 = None; self.measure_e1 = None; self.measure_result = None
        self.measure_var.set("")
        self.redraw()

    # ---------- cleanup dialog ----------
    def cleanup_dialog(self):
        if not self.doc:
            messagebox.showwarning("Нет файла", "Откройте DXF"); return
        dlg = tk.Toplevel(self.root)
        dlg.title("Очистить мусор")
        dlg.geometry("460x340")
        dlg.transient(self.root); dlg.grab_set()

        tk.Label(dlg, text="Что удалить перед слиянием:",
                 anchor='w', font=('Arial', 10, 'bold')).pack(fill='x', padx=10, pady=(10, 4))
        vars = {}
        for key, txt, default in [
            ('zero_length', 'Нулевой длины (< допуска)', True),
            ('isolated', 'Изолированные (
