#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DXF Contour Fixer v3
- Сливает связанные сегменты в ОДИН замкнутый LWPOLYLINE (как CorelDRAW Join Curves)
- Экспорт чистого DXF и PLT (HPGL) — для EZCAD
- GUI с превью и красными точками разрывов

Зависимость:  pip install ezdxf
"""

import os, sys, tkinter as tk
from tkinter import filedialog, messagebox
from math import pi, cos, sin, radians, hypot, floor

try:
    import ezdxf
    from ezdxf.math import Vec2, bulge_to_arc
except ImportError:
    _r = tk.Tk(); _r.withdraw()
    messagebox.showerror("Missing ezdxf", "pip install ezdxf")
    sys.exit(1)


# ==================== geometry ====================

def entity_full_path(e):
    """Возвращает список Vec2 — полилинейное представление примитива."""
    t = e.dxftype()
    try:
        if t == 'LINE':
            return [Vec2(e.dxf.start.x, e.dxf.start.y),
                    Vec2(e.dxf.end.x, e.dxf.end.y)]
        if t == 'ARC':
            cx, cy, r = e.dxf.center.x, e.dxf.center.y, e.dxf.radius
            a1, a2 = radians(e.dxf.start_angle), radians(e.dxf.end_angle)
            if a2 <= a1: a2 += 2*pi
            n = max(12, int((a2 - a1) / radians(5)))
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
            if e.closed and (pts[0] - pts[-1]).magnitude > 1e-12:
                pts.append(pts[0])
            return pts
        if t == 'POLYLINE':
            pts = [Vec2(v.dxf.location.x, v.dxf.location.y) for v in e.vertices]
            if e.is_closed and pts and (pts[0] - pts[-1]).magnitude > 1e-12:
                pts.append(pts[0])
            return pts if len(pts) >= 2 else None
        if t == 'SPLINE':
            pts = [Vec2(p.x, p.y) for p in e.flattening(0.1)]
            return pts if len(pts) >= 2 else None
    except Exception:
        return None
    return None


def signed_area(pts):
    s = 0.0
    for i in range(len(pts)-1):
        s += pts[i].x * pts[i+1].y - pts[i+1].x * pts[i].y
    return s * 0.5


# ==================== merging (CorelDRAW style) ====================

def merge_to_contours(doc, tol, log):
    """Возвращает список замкнутых контуров (list[Vec2], first == last)."""
    msp = doc.modelspace()
    segs, preserved = [], []
    for e in msp:
        t = e.dxftype()
        if t == 'CIRCLE':
            p = entity_full_path(e)
            if p: preserved.append(p)
            continue
        if t in ('LWPOLYLINE', 'POLYLINE'):
            is_closed = (t == 'LWPOLYLINE' and e.closed) or (t == 'POLYLINE' and e.is_closed)
            if is_closed:
                p = entity_full_path(e)
                if p: preserved.append(p)
                continue
        p = entity_full_path(e)
        if p and len(p) >= 2:
            segs.append(p)
    
    log(f"  Открытых сегментов: {len(segs)} | уже замкнутых: {len(preserved)}")
    if not segs:
        return preserved

    # --- узлы ---
    nodes, buckets = [], {}
    cell = max(tol*2, 1e-9)
    def get_node(p):
        key = (floor(p.x/cell), floor(p.y/cell))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for i in buckets.get((key[0]+dx, key[1]+dy), ()):
                    if (nodes[i]-p).magnitude < tol:
                        return i
        nodes.append(p)
        buckets.setdefault(key, []).append(len(nodes)-1)
        return len(nodes)-1

    edges, adj = [], {}
    for seg in segs:
        a = get_node(seg[0]); b = get_node(seg[-1])
        if a == b:
            preserved.append([nodes[a]] + list(seg[1:-1]) + [nodes[a]])
            continue
        snapped = [nodes[a]] + list(seg[1:-1]) + [nodes[b]]
        idx = len(edges)
        edges.append({'a': a, 'b': b, 'pts': snapped, 'used': False})
        adj.setdefault(a, []).append(idx)
        adj.setdefault(b, []).append(idx)

    # --- union-find компонент ---
    parent = list(range(len(nodes)))
    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]; x = parent[x]
        return x
    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb: parent[ra] = rb
    for ed in edges: union(ed['a'], ed['b'])
    comps = {}
    for n in adj: comps.setdefault(find(n), []).append(n)

    # --- трассировка ---
    traced = []
    for root, members in comps.items():
        start = None
        for n in members:
            if sum(1 for ei in adj.get(n, []) if not edges[ei]['used']) == 1:
                start = n; break
        if start is None: start = members[0]
        tried = set()

        while True:
            avail = [ei for ei in adj.get(start, []) if not edges[ei]['used']]
            if not avail:
                found = None
                for n in members:
                    if n in tried: continue
                    for ei in adj.get(n, []):
                        if not edges[ei]['used']:
                            found = n; break
                    if found is not None: break
                if found is None: break
                start = found
            tried.add(start)

            path = [nodes[start]]
            cur = start
            while True:
                av = [ei for ei in adj.get(cur, []) if not edges[ei]['used']]
                if not av: break
                if len(path) >= 2:
                    prev, last = path[-2], path[-1]
                    def rank(ei):
                        ed = edges[ei]
                        other = ed['b'] if ed['a'] == cur else ed['a']
                        tgt = nodes[other]
                        v1x, v1y = last.x-prev.x, last.y-prev.y
                        v2x, v2y = tgt.x-last.x, tgt.y-last.y
                        m = hypot(v1x, v1y)*hypot(v2x, v2y)
                        if m < 1e-12: return 1.0
                        return -(v1x*v2x + v1y*v2y)/m  # меньше = «прямее»
                    av.sort(key=rank)
                ei = av[0]
                ed = edges[ei]; ed['used'] = True
                if ed['a'] == cur:
                    path.extend(ed['pts'][1:]); cur = ed['b']
                else:
                    path.extend(list(reversed(ed['pts']))[1:]); cur = ed['a']

            if (path[-1] - path[0]).magnitude > tol:
                path.append(path[0])
                log(f"  Замкнут контур: {len(path)} точек")
            else:
                path[-1] = path[0]

            if len(path) >= 3:
                traced.append(path)

    return preserved + traced


# ==================== writers ====================

def write_clean_dxf(contours, out_path, all_ccw=False):
    doc = ezdxf.new('R2010')
    msp = doc.modelspace()
    doc.layers.add("CONTOUR", color=7)
    n = 0
    for pts in contours:
        if len(pts) < 3: continue
        if all_ccw and signed_area(pts) < 0:
            pts = list(reversed(pts))
        coords = ([(p.x, p.y) for p in pts[:-1]]
                  if (pts[0]-pts[-1]).magnitude < 1e-9
                  else [(p.x, p.y) for p in pts])
        msp.add_lwpolyline(coords, close=True, dxfattribs={'layer': 'CONTOUR'})
        n += 1
    doc.saveas(out_path)
    return n


def write_plt(contours, out_path, units_per_mm=40.0, margin_mm=1.0):
    xs = [p.x for pts in contours for p in pts]
    ys = [p.y for pts in contours for p in pts]
    if not xs: raise ValueError("Нет контуров")
    minx, miny = min(xs), min(ys)
    margin = margin_mm * units_per_mm

    def hp(p):
        return (int(round((p.x - minx)*units_per_mm + margin)),
                int(round((p.y - miny)*units_per_mm + margin)))

    lines = ["IN;", "PA;", "SP1;"]
    n = 0
    for pts in contours:
        if len(pts) < 3: continue
        if (pts[0]-pts[-1]).magnitude > 1e-9:
            pts = list(pts) + [pts[0]]
        h = [hp(p) for p in pts]
        x0, y0 = h[0]
        lines.append(f"PU{x0},{y0};")
        CH = 20
        for i in range(1, len(h), CH):
            chunk = h[i:i+CH]
            lines.append("PD" + ",".join(f"{x},{y}" for x, y in chunk) + ";")
        lines.append("PU;")
        n += 1
    lines.append("PG;")
    with open(out_path, 'w', newline='\r\n') as f:
        f.write("\n".join(lines))
    return n


# ==================== GUI ====================

class DxfApp:
    def __init__(self, root):
        self.root = root
        root.title("DXF Contour Fixer v3")
        root.geometry("1200x820")

        self.doc = None
        self.file_path = None
        self.contours = None    # результат слияния
        self.render = []
        self.break_points = []
        self.bbox = None
        self.scale = 1.0
        self.ox = 0.0
        self.oy = 0.0
        self._pan = None

        self._build_ui()

    def _build_ui(self):
        tb = tk.Frame(self.root, bd=1, relief='raised'); tb.pack(side='top', fill='x')
        tk.Button(tb, text="1. Открыть DXF", command=self.open_file).pack(side='left', padx=4, pady=4)
        tk.Button(tb, text="2. Найти разрывы", command=self.find_breaks).pack(side='left', padx=2)
        tk.Button(tb, text="3. Слить в контуры", command=self.do_fix).pack(side='left', padx=2)
        tk.Button(tb, text="4. Сохранить DXF", command=self.save_dxf).pack(side='left', padx=2)
        tk.Button(tb, text="5. Сохранить PLT", command=self.save_plt).pack(side='left', padx=2)
        tk.Button(tb, text="Вписать", command=self.fit_view).pack(side='left', padx=8)

        tk.Label(tb, text="Допуск:").pack(side='left', padx=(10, 2))
        self.tol_var = tk.StringVar(value="0.01")
        tk.Entry(tb, textvariable=self.tol_var, width=8).pack(side='left')

        self.ccw_var = tk.BooleanVar(value=False)
        tk.Checkbutton(tb, text="Все CCW", variable=self.ccw_var).pack(side='left', padx=6)

        paned = tk.PanedWindow(self.root, orient='vertical', sashwidth=6)
        paned.pack(fill='both', expand=True)
        cf = tk.Frame(paned); self.canvas = tk.Canvas(cf, bg='white')
        self.canvas.pack(fill='both', expand=True)
        paned.add(cf, stretch='always')
        lf = tk.Frame(paned)
        tk.Label(lf, text="Лог:", anchor='w').pack(fill='x')
        self.log_text = tk.Text(lf, height=8, font=('Courier', 9))
        self.log_text.pack(fill='both', expand=True)
        paned.add(lf)

        self.status_var = tk.StringVar(value="Откройте DXF.")
        tk.Label(self.root, textvariable=self.status_var, anchor='w',
                 bd=1, relief='sunken').pack(fill='x', side='bottom')

        self.canvas.bind('<Configure>', lambda e: self.redraw())
        for ev in ('<MouseWheel>', '<Button-4>', '<Button-5>'):
            self.canvas.bind(ev, self._wheel)
        for ev in ('<ButtonPress-1>', '<ButtonPress-2>'):
            self.canvas.bind(ev, self._pan_start)
        for ev in ('<B1-Motion>', '<B2-Motion>'):
            self.canvas.bind(ev, self._pan_move)
        for ev in ('<ButtonRelease-1>', '<ButtonRelease-2>'):
            self.canvas.bind(ev, self._pan_end)

    # ---- utils ----
    def log(self, m):
        self.log_text.insert('end', m+'\n'); self.log_text.see('end'); self.root.update_idletasks()
    def status(self, m):
        self.status_var.set(m); self.root.update_idletasks()
    def get_tol(self):
        try:
            t = float(self.tol_var.get())
            if t <= 0: raise ValueError
            return t
        except ValueError:
            messagebox.showerror("Ошибка", "Допуск — положительное число"); return None

    # ---- открытие ----
    def open_file(self):
        p = filedialog.askopenfilename(filetypes=[("DXF", "*.dxf"), ("All", "*.*")])
        if not p: return
        try:
            self.status("Загрузка…")
            self.doc = ezdxf.readfile(p)
            self.file_path = p
            self.contours = None
            self.break_points = []
            self.log_text.delete('1.0', 'end')
            self.log(f"Загружено: {p}")
            self._render_doc()
            self.fit_view()
            self.status(f"Загружено. Нажмите «Найти разрывы».")
        except Exception as ex:
            messagebox.showerror("Ошибка", str(ex)); self.status("Ошибка")

    def _render_doc(self):
        self.render = []
        if not self.doc: return
        x0=y0=float('inf'); x1=y1=float('-inf')
        for e in self.doc.modelspace():
            for pl in [entity_full_path(e)]:
                if not pl or len(pl) < 2: continue
                xy = [(p.x, p.y) for p in pl]
                self.render.append((xy, 'black'))
                for x, y in xy:
                    if x < x0: x0 = x
                    if x > x1: x1 = x
                    if y < y0: y0 = y
                    if y > y1: y1 = y
        self.bbox = (x0, y0, x1, y1) if x0 < float('inf') else None

    def _render_contours(self):
        self.render = []
        x0=y0=float('inf'); x1=y1=float('-inf')
        for pts in self.contours or []:
            xy = [(p.x, p.y) for p in pts]
            self.render.append((xy, '#0033cc'))
            for x, y in xy:
                if x < x0: x0 = x
                if x > x1: x1 = x
                if y < y0: y0 = y
                if y > y1: y1 = y
        self.bbox = (x0, y0, x1, y1) if x0 < float('inf') else None

    # ---- разрывы ----
    def find_breaks(self):
        if not self.doc:
            messagebox.showwarning("Нет файла", "Откройте DXF"); return
        tol = self.get_tol()
        if tol is None: return
        self.contours = None
        self._render_doc()
        self.log("Поиск разрывов…")
        self.break_points = self._compute_breaks(tol)
        self.log(f"Разрывов: {len(self.break_points)}")
        self.redraw()
        self.status(f"Красных точек (разрывов): {len(self.break_points)}")

    def _compute_breaks(self, tol):
        pts = []
        for e in self.doc.modelspace():
            p = entity_full_path(e)
            if not p or len(p) < 2: continue
            if (p[0]-p[-1]).magnitude < tol:
                continue  # сам примитив уже замкнут
            pts.append(p[0]); pts.append(p[-1])
        if not pts: return []
        # кластеризация
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
        return [(pts[g[0]].x, pts[g[0]].y) for g in groups.values() if len(g) == 1]

    # ---- слияние ----
    def do_fix(self):
        if not self.doc:
            messagebox.showwarning("Нет файла", "Откройте DXF"); return
        tol = self.get_tol()
        if tol is None: return
        try:
            self.log(f"Слияние (допуск {tol})…")
            self.contours = merge_to_contours(self.doc, tol, self.log)
            self.log(f"Итого контуров: {len(self.contours)}")
            self._render_contours()
            self.fit_view()
            self.break_points = []
            self.status(f"Слито контуров: {len(self.contours)}. Разрывов: 0 (все замкнуты)")
            self.log("Готово. Теперь «Сохранить DXF» и/или «Сохранить PLT».")
        except Exception as ex:
            messagebox.showerror("Ошибка", str(ex)); self.log(f"ОШИБКА: {ex}")

    # ---- сохранение ----
    def save_dxf(self):
        if not self.contours:
            messagebox.showwarning("Нет данных", "Сначала «Слить в контуры»"); return
        default = os.path.splitext(os.path.basename(self.file_path or "output"))[0] + "_clean.dxf"
        out = filedialog.asksaveasfilename(defaultextension=".dxf",
                                            filetypes=[("DXF", "*.dxf")],
                                            initialfile=default)
        if not out: return
        try:
            n = write_clean_dxf(self.contours, out, all_ccw=self.ccw_var.get())
            self.log(f"DXF сохранён: {out} ({n} LWPOLYLINE)")
            self.status(f"DXF сохранён: {n} контуров")
            messagebox.showinfo("Готово", f"Сохранено {n} замкнутых LWPOLYLINE в:\n{out}")
        except Exception as ex:
            messagebox.showerror("Ошибка", str(ex))

    def save_plt(self):
        if not self.contours:
            messagebox.showwarning("Нет данных", "Сначала «Слить в контуры»"); return
        default = os.path.splitext(os.path.basename(self.file_path or "output"))[0] + ".plt"
        out = filedialog.asksaveasfilename(defaultextension=".plt",
                                            filetypes=[("PLT/HPGL", "*.plt"), ("HPGL", "*.hpgl")],
                                            initialfile=default)
        if not out: return
        try:
            n = write_plt(self.contours, out)
            self.log(f"PLT сохранён: {out} ({n} контуров)")
            self.status(f"PLT сохранён: {n} контуров")
            messagebox.showinfo("Готово", f"Сохранено {n} контуров в PLT:\n{out}")
        except Exception as ex:
            messagebox.showerror("Ошибка", str(ex))

    # ---- вид ----
    def fit_view(self):
        if not self.bbox: return
        x0, y0, x1, y1 = self.bbox
        w, h = max(x1-x0, 1e-6), max(y1-y0, 1e-6)
        cw, ch = self.canvas.winfo_width(), self.canvas.winfo_height()
        if cw < 10 or ch < 10:
            self.root.after(50, self.fit_view); return
        m = 0.05
        self.scale = min((cw*(1-2*m))/w, (ch*(1-2*m))/h)
        self.ox = (x0+x1)/2 - cw/(2*self.scale)
        self.oy = (y0+y1)/2 - ch/(2*self.scale)
        self.redraw()

    def w2s(self, x, y):
        ch = self.canvas.winfo_height()
        return (x - self.ox)*self.scale, ch - (y - self.oy)*self.scale

    def redraw(self):
        self.canvas.delete('all')
        for pl, col in self.render:
            sp = []
            for x, y in pl:
                sx, sy = self.w2s(x, y); sp.extend([sx, sy])
            if len(sp) >= 4:
                self.canvas.create_line(*sp, fill=col, width=1)
        r = 6
        for x, y in self.break_points:
            sx, sy = self.w2s(x, y)
            self.canvas.create_oval(sx-r, sy-r, sx+r, sy+r,
                                     fill='red', outline='darkred', width=2)

    def _wheel(self, ev):
        if not self.bbox: return
        up = (getattr(ev, 'delta', 0) > 0) or (getattr(ev, 'num', 0) == 4)
        dn = (getattr(ev, 'delta', 0) < 0) or (getattr(ev, 'num', 0) == 5)
        if not up and not dn: return
        f = 1.2 if up else 1/1.2
        ch = self.canvas.winfo_height()
        wx = ev.x/self.scale + self.ox
        wy = (ch-ev.y)/self.scale + self.oy
        self.scale *= f
        self.ox = wx - ev.x/self.scale
        self.oy = wy - (ch-ev.y)/self.scale
        self.redraw()

    def _pan_start(self, ev): self._pan = (ev.x, ev.y)
    def _pan_move(self, ev):
        if self._pan is None: return
        dx, dy = ev.x - self._pan[0], ev.y - self._pan[1]
        self.ox -= dx/self.scale
        self.oy += dy/self.scale
        self._pan = (ev.x, ev.y)
        self.redraw()
    def _pan_end(self, ev): self._pan = None


if __name__ == "__main__":
    root = tk.Tk(); DxfApp(root); root.mainloop()
