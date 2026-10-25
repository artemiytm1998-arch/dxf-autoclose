#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DXF Contour Fixer v2
- Снапит близкие концы в одну точку (лечит проблему с EZCAD)
- Замыкает незамкнутые контуры
- Графическое превью с показом точек разрыва (красные кружки)

Зависимость:  pip install ezdxf
"""

import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox
from math import atan2, degrees, radians, cos, sin, hypot, floor, pi, tan

try:
    import ezdxf
    from ezdxf.math import Vec2, bulge_to_arc
except ImportError:
    messagebox.showerror("Missing ezdxf", "Установите: pip install ezdxf")
    sys.exit(1)


# ==================== Геометрия ====================

def entity_endpoints(e):
    """(start, end) для открытого примитива, None для замкнутого."""
    t = e.dxftype()
    try:
        if t == 'LINE':
            return Vec2(e.dxf.start.x, e.dxf.start.y), Vec2(e.dxf.end.x, e.dxf.end.y)
        if t == 'ARC':
            return Vec2(e.start_point.x, e.start_point.y), Vec2(e.end_point.x, e.end_point.y)
        if t == 'LWPOLYLINE':
            if e.closed: return None
            pts = list(e.get_points('xy'))
            return (Vec2(*pts[0]), Vec2(*pts[-1])) if len(pts) >= 2 else None
        if t == 'POLYLINE':
            if e.is_closed: return None
            vs = list(e.vertices)
            if len(vs) < 2: return None
            return Vec2(vs[0].dxf.location.x, vs[0].dxf.location.y), \
                   Vec2(vs[-1].dxf.location.x, vs[-1].dxf.location.y)
        if t == 'SPLINE':
            return Vec2(e.start_point.x, e.start_point.y), Vec2(e.end_point.x, e.end_point.y)
    except Exception:
        return None
    return None


def convert_arc_to_lwpolyline(msp, arc):
    """ARC -> LWPOLYLINE с bulge (сохраняет форму дуги)."""
    try:
        cx, cy = arc.dxf.center.x, arc.dxf.center.y
        r = arc.dxf.radius
        a1 = arc.dxf.start_angle % 360
        a2 = arc.dxf.end_angle % 360
        if a2 <= a1:
            a2 += 360
        bulge = tan(radians(a2 - a1) / 4)
        ar1, ar2 = radians(a1), radians(a2)
        p1 = (cx + r * cos(ar1), cy + r * sin(ar1))
        p2 = (cx + r * cos(ar2), cy + r * sin(ar2))
        lwp = msp.add_lwpolyline([(p1[0], p1[1], bulge), (p2[0], p2[1])], format='xyb')
        lwp.dxf.layer = arc.dxf.layer
        return lwp
    except Exception:
        return None


def _union_find_indices(n):
    parent = list(range(n))
    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb: parent[ra] = rb
    return parent, find, union


def cluster_points(points, tolerance):
    """Возвращает список кластеров (каждый — список индексов в points)."""
    n = len(points)
    if n == 0: return []
    parent, find, union = _union_find_indices(n)
    cell = max(tolerance * 2, 1e-9)
    buckets = {}
    for i, p in enumerate(points):
        key = (floor(p.x / cell), floor(p.y / cell))
        buckets.setdefault(key, []).append(i)
    for i, p in enumerate(points):
        key = (floor(p.x / cell), floor(p.y / cell))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j in buckets.get((key[0] + dx, key[1] + dy), ()):
                    if j <= i: continue
                    if (points[j] - p).magnitude <= tolerance:
                        union(i, j)
    clusters = {}
    for i in range(n):
        clusters.setdefault(find(i), []).append(i)
    return list(clusters.values())


# ==================== Анализ разрывов ====================

def find_breaks(doc, tolerance):
    """Точки-«тупики»: концы, у которых нет соседа в пределах допуска."""
    pts = []
    for e in doc.modelspace():
        try:
            ep = entity_endpoints(e)
        except Exception:
            continue
        if ep is None: continue
        pts.append(ep[0])
        pts.append(ep[1])
    if not pts: return []
    result = []
    for cluster in cluster_points(pts, tolerance):
        if len(cluster) == 1:
            p = pts[cluster[0]]
            result.append((p.x, p.y))
    return result


# ==================== Основная обработка ====================

def fix_contours(doc, tolerance, aggressive, log):
    msp = doc.modelspace()
    if "AUTOCLOSED" not in doc.layers:
        doc.layers.add("AUTOCLOSED", color=1)

    # 1) ARC -> LWPOLYLINE (у дуг нельзя напрямую двигать концы)
    arcs_to_delete = []
    n_conv = 0
    for e in list(msp):
        if e.dxftype() == 'ARC':
            new = convert_arc_to_lwpolyline(msp, e)
            if new is not None:
                arcs_to_delete.append(e)
                n_conv += 1
    for e in arcs_to_delete:
        try: msp.delete_entity(e)
        except Exception: pass
    if n_conv:
        log(f"  Конвертировано дуг в полилинии: {n_conv}")

    # 2) Собираем записи об открытых примитивах
    records = []  # {'e', 'start', 'end'}
    for e in msp:
        t = e.dxftype()
        if t == 'CIRCLE': continue
        if t in ('LWPOLYLINE', 'POLYLINE'):
            if (t == 'LWPOLYLINE' and e.closed) or (t == 'POLYLINE' and e.is_closed):
                continue
        try:
            ep = entity_endpoints(e)
        except Exception:
            continue
        if ep is None: continue
        records.append({'e': e, 'start': ep[0], 'end': ep[1]})

    # 3) Собираем все концы и кластеризуем
    all_pts = []  # (rec_idx, 'S'|'E', Vec2)
    for i, r in enumerate(records):
        all_pts.append((i, 'S', r['start']))
        all_pts.append((i, 'E', r['end']))

    point_vecs = [p for _, _, p in all_pts]
    clusters = cluster_points(point_vecs, tolerance)

    # 4) Снапим: точки кластера -> их среднее
    n_snapped = 0
    for cluster in clusters:
        if len(cluster) < 2:
            continue
        cx = sum(point_vecs[i].x for i in cluster) / len(cluster)
        cy = sum(point_vecs[i].y for i in cluster) / len(cluster)
        target = Vec2(cx, cy)
        for i in cluster:
            rec_i, which, _ = all_pts[i]
            if which == 'S':
                records[rec_i]['start'] = target
            else:
                records[rec_i]['end'] = target
            n_snapped += 1

    # 5) Записываем снапнутые координаты обратно в примитивы
    for rec in records:
        e = rec['e']
        t = e.dxftype()
        try:
            if t == 'LINE':
                e.dxf.start = (rec['start'].x, rec['start'].y, e.dxf.start.z)
                e.dxf.end = (rec['end'].x, rec['end'].y, e.dxf.end.z)
            elif t == 'LWPOLYLINE':
                pts = list(e.get_points('xyb'))
                pts[0] = (rec['start'].x, rec['start'].y, pts[0][2])
                pts[-1] = (rec['end'].x, rec['end'].y, pts[-1][2])
                e.set_points(pts, format='xyb')
            elif t == 'POLYLINE':
                vs = list(e.vertices)
                vs[0].dxf.location = (rec['start'].x, rec['start'].y, vs[0].dxf.location.z)
                vs[-1].dxf.location = (rec['end'].x, rec['end'].y, vs[-1].dxf.location.z)
        except Exception as ex:
            log(f"  ! сбой снапа {t}: {ex}")

    # 6) Удаляем схлопнувшиеся (start == end)
    dead = [r for r in records if (r['start'] - r['end']).magnitude < 1e-9]
    for r in dead:
        try: msp.delete_entity(r['e'])
        except Exception: pass
        records.remove(r)
    if dead:
        log(f"  Удалено нулевых примитивов: {len(dead)}")

    # 7) Граф по снапнутым концам
    nodes, buckets = [], {}
    cell = max(tolerance * 2, 1e-9)
    def get_node(p):
        key = (floor(p.x / cell), floor(p.y / cell))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for i in buckets.get((key[0] + dx, key[1] + dy), ()):
                    if (nodes[i] - p).magnitude < tolerance:
                        return i
        nodes.append(p)
        buckets.setdefault(key, []).append(len(nodes) - 1)
        return len(nodes) - 1

    edges = []
    for r in records:
        a, b = get_node(r['start']), get_node(r['end'])
        if a != b: edges.append((a, b))

    n_nodes = len(nodes)
    if n_nodes == 0:
        return n_snapped, 0, 0
    parent2, find2, union2 = _union_find_indices(n_nodes)
    for a, b in edges: union2(a, b)
    comps = {}
    for i in range(n_nodes): comps.setdefault(find2(i), []).append(i)
    deg = [0] * n_nodes
    for a, b in edges: deg[a] += 1; deg[b] += 1

    # 8) Замыкающие линии
    n_added = 0
    n_failed = 0
    for root, members in comps.items():
        if len(members) == 1 and deg[members[0]] == 0:
            continue
        open_nodes = [n for n in members if deg[n] == 1]
        if not open_nodes: continue
        if len(open_nodes) == 2:
            p1, p2 = nodes[open_nodes[0]], nodes[open_nodes[1]]
            ln = msp.add_line((p1.x, p1.y, 0), (p2.x, p2.y, 0))
            ln.dxf.layer = "AUTOCLOSED"
            n_added += 1
        elif aggressive:
            rem = list(open_nodes)
            while len(rem) >= 2:
                best = None
                for i in range(len(rem)):
                    for j in range(i + 1, len(rem)):
                        d = (nodes[rem[i]] - nodes[rem[j]]).magnitude
                        if best is None or d < best[0]:
                            best = (d, i, j)
                _, i, j = best
                p1, p2 = nodes[rem[i]], nodes[rem[j]]
                ln = msp.add_line((p1.x, p1.y, 0), (p2.x, p2.y, 0))
                ln.dxf.layer = "AUTOCLOSED"
                n_added += 1
                for k in sorted([i, j], reverse=True): rem.pop(k)
            if rem: n_failed += 1
        else:
            n_failed += 1
    return n_snapped, n_added, n_failed


# ==================== Превью ====================

def entity_to_polylines(e):
    """Список ломаных [(x,y),...] для отрисовки."""
    t = e.dxftype()
    try:
        if t == 'LINE':
            return [[(e.dxf.start.x, e.dxf.start.y), (e.dxf.end.x, e.dxf.end.y)]]
        if t == 'CIRCLE':
            cx, cy, r = e.dxf.center.x, e.dxf.center.y, e.dxf.radius
            n = 64
            return [[(cx + r*cos(2*pi*i/n), cy + r*sin(2*pi*i/n)) for i in range(n+1)]]
        if t == 'ARC':
            cx, cy, r = e.dxf.center.x, e.dxf.center.y, e.dxf.radius
            a1, a2 = radians(e.dxf.start_angle), radians(e.dxf.end_angle)
            if a2 <= a1: a2 += 2*pi
            n = 48
            return [[(cx + r*cos(a1+(a2-a1)*i/n), cy + r*sin(a1+(a2-a1)*i/n)) for i in range(n+1)]]
        if t == 'LWPOLYLINE':
            raw = list(e.get_points('xyb'))
            if len(raw) < 2: return []
            pts = []
            for i, (x, y, b) in enumerate(raw):
                pts.append((x, y))
                if b != 0 and i < len(raw) - 1:
                    nxt = raw[i+1]
                    try:
                        c, sa, ea, r = bulge_to_arc(Vec2(x, y), Vec2(nxt[0], nxt[1]), b)
                    except Exception:
                        continue
                    if b > 0:
                        if ea <= sa: ea += 2*pi
                    else:
                        if ea >= sa: ea -= 2*pi
                    n = 16
                    for k in range(1, n):
                        a = sa + (ea - sa) * k / n
                        pts.append((c.x + r*cos(a), c.y + r*sin(a)))
            if e.closed and pts: pts.append(pts[0])
            return [pts]
        if t == 'POLYLINE':
            pts = [(v.dxf.location.x, v.dxf.location.y) for v in e.vertices]
            if e.is_closed and pts: pts.append(pts[0])
            return [pts]
        if t == 'SPLINE':
            try:
                return [[(p.x, p.y) for p in e.flattening(0.2)]]
            except Exception:
                return []
    except Exception:
        return []
    return []


# ==================== GUI ====================

class DxfApp:
    def __init__(self, root):
        self.root = root
        root.title("DXF Contour Fixer v2")
        root.geometry("1200x800")

        self.doc = None
        self.file_path = None
        self.render = []        # [(list_of_points, color)]
        self.break_points = []
        self.bbox = None
        self.scale = 1.0
        self.ox = 0.0
        self.oy = 0.0
        self._pan_last = None

        self._build_ui()

    def _build_ui(self):
        tb = tk.Frame(self.root, bd=1, relief='raised')
        tb.pack(side='top', fill='x')

        tk.Button(tb, text="Открыть DXF…", command=self.open_file).pack(side='left', padx=4, pady=4)
        tk.Button(tb, text="Найти разрывы", command=self.find_breaks).pack(side='left', padx=2)
        tk.Button(tb, text="Исправить и сохранить…", command=self.fix_and_save).pack(side='left', padx=2)
        tk.Button(tb, text="Вписать в окно", command=self.fit_view).pack(side='left', padx=8)

        tk.Label(tb, text="Допуск:").pack(side='left', padx=(12, 2))
        self.tol_var = tk.StringVar(value="0.01")
        tk.Entry(tb, textvariable=self.tol_var, width=10).pack(side='left')
        self.aggr_var = tk.BooleanVar(value=False)
        tk.Checkbutton(tb, text="Aggressive", variable=self.aggr_var).pack(side='left', padx=6)

        paned = tk.PanedWindow(self.root, orient='vertical', sashwidth=6)
        paned.pack(fill='both', expand=True)

        cf = tk.Frame(paned)
        self.canvas = tk.Canvas(cf, bg='white')
        self.canvas.pack(fill='both', expand=True)
        paned.add(cf, stretch='always')

        lf = tk.Frame(paned)
        tk.Label(lf, text="Лог:", anchor='w').pack(fill='x')
        self.log_text = tk.Text(lf, height=8, font=('Courier', 9), wrap='word')
        self.log_text.pack(fill='both', expand=True)
        paned.add(lf)

        self.status_var = tk.StringVar(value="Откройте DXF для начала.")
        tk.Label(self.root, textvariable=self.status_var, anchor='w',
                 bd=1, relief='sunken').pack(fill='x', side='bottom')

        self.canvas.bind('<Configure>', lambda e: self.redraw())
        self.canvas.bind('<MouseWheel>', self._wheel)
        self.canvas.bind('<Button-4>', self._wheel)
        self.canvas.bind('<Button-5>', self._wheel)
        self.canvas.bind('<ButtonPress-1>', self._pan_start)
        self.canvas.bind('<B1-Motion>', self._pan_move)
        self.canvas.bind('<ButtonRelease-1>', self._pan_end)
        self.canvas.bind('<ButtonPress-2>', self._pan_start)
        self.canvas.bind('<B2-Motion>', self._pan_move)
        self.canvas.bind('<ButtonRelease-2>', self._pan_end)

    # -------- служебное --------
    def log(self, msg):
        self.log_text.insert('end', msg + '\n'); self.log_text.see('end')
        self.root.update_idletasks()

    def status(self, msg):
        self.status_var.set(msg); self.root.update_idletasks()

    def get_tol(self):
        try:
            t = float(self.tol_var.get())
            if t <= 0: raise ValueError
            return t
        except ValueError:
            messagebox.showerror("Ошибка", "Допуск должен быть положительным числом")
            return None

    # -------- файлы --------
    def open_file(self):
        path = filedialog.askopenfilename(filetypes=[("DXF", "*.dxf"), ("Все файлы", "*.*")])
        if not path: return
        try:
            self.status("Загрузка…")
            self.doc = ezdxf.readfile(path)
            self.file_path = path
            self.log_text.delete('1.0', 'end')
            self.log(f"Загружено: {path}")
            self._build_render()
            self.break_points = []
            self.fit_view()
            self.status(f"Загружено. Нажмите «Найти разрывы».")
        except Exception as ex:
            messagebox.showerror("Ошибка", str(ex))
            self.status("Ошибка загрузки")

    def _build_render(self):
        self.render = []
        if not self.doc: return
        x0 = y0 = float('inf'); x1 = y1 = float('-inf')
        for e in self.doc.modelspace():
            layer = e.dxf.get('layer', '0')
            color = '#008800' if layer == "AUTOCLOSED" else 'black'
            for pl in entity_to_polylines(e):
                if len(pl) < 2: continue
                self.render.append((pl, color))
                for x, y in pl:
                    if x < x0: x0 = x
                    if x > x1: x1 = x
                    if y < y0: y0 = y
                    if y > y1: y1 = y
        self.bbox = (x0, y0, x1, y1) if x0 < float('inf') else None

    # -------- разрывы --------
    def find_breaks(self):
        if not self.doc:
            messagebox.showwarning("Нет файла", "Сначала откройте DXF"); return
        tol = self.get_tol()
        if tol is None: return
        self.log(f"Поиск разрывов (допуск {tol})…")
        self.break_points = find_breaks(self.doc, tol)
        self.log(f"Найдено разрывов: {len(self.break_points)}")
        self.redraw()
        self.status(f"Разрывов: {len(self.break_points)}")

    # -------- исправление --------
    def fix_and_save(self):
        if not self.doc:
            messagebox.showwarning("Нет файла", "Сначала откройте DXF"); return
        tol = self.get_tol()
        if tol is None: return
        default = os.path.splitext(os.path.basename(self.file_path or "output"))[0] + "_fixed.dxf"
        out = filedialog.asksaveasfilename(defaultextension=".dxf",
                                            filetypes=[("DXF", "*.dxf")],
                                            initialfile=default)
        if not out: return
        self.log(f"Исправление (допуск {tol})…")
        try:
            ns, na, nf = fix_contours(self.doc, tol, self.aggr_var.get(), self.log)
            self.log(f"Снапнуто концов: {ns} | Добавлено замыканий: {na} | Не удалось: {nf}")
            self.doc.saveas(out)
            self.log(f"Сохранено: {out}")
            # Перечитать сохранённый файл — чтобы показать уже исправленное
            self.doc = ezdxf.readfile(out)
            self.file_path = out
            self._build_render()
            self.break_points = find_breaks(self.doc, tol)
            self.fit_view()
            self.status(f"Сохранено. Осталось разрывов: {len(self.break_points)}")
        except Exception as ex:
            messagebox.showerror("Ошибка", str(ex))
            self.log(f"ОШИБКА: {ex}")

    # -------- вид --------
    def fit_view(self):
        if not self.bbox:
            return
        x0, y0, x1, y1 = self.bbox
        w = max(x1 - x0, 1e-6); h = max(y1 - y0, 1e-6)
        cw = self.canvas.winfo_width(); ch = self.canvas.winfo_height()
        if cw < 10 or ch < 10:
            self.root.after(50, self.fit_view); return
        margin = 0.05
        s = min((cw * (1 - 2*margin)) / w, (ch * (1 - 2*margin)) / h)
        self.scale = s
        self.ox = (x0 + x1)/2 - cw / (2*s)
        self.oy = (y0 + y1)/2 - ch / (2*s)
        self.redraw()

    def w2s(self, x, y):
        ch = self.canvas.winfo_height()
        return (x - self.ox) * self.scale, ch - (y - self.oy) * self.scale

    def redraw(self):
        self.canvas.delete('all')
        for pl, color in self.render:
            sp = []
            for x, y in pl:
                sx, sy = self.w2s(x, y)
                sp.extend([sx, sy])
            if len(sp) >= 4:
                self.canvas.create_line(*sp, fill=color, width=1)
        r = 6
        for x, y in self.break_points:
            sx, sy = self.w2s(x, y)
            self.canvas.create_oval(sx - r, sy - r, sx + r, sy + r,
                                     fill='red', outline='darkred', width=2)

    def _wheel(self, event):
        if not self.bbox: return
        up = (getattr(event, 'delta', 0) > 0) or (getattr(event, 'num', 0) == 4)
        if not up and not ((getattr(event, 'delta', 0) < 0) or (getattr(event, 'num', 0) == 5)):
            return
        factor = 1.2 if up else 1/1.2
        sx, sy = event.x, event.y
        ch = self.canvas.winfo_height()
        wx = sx / self.scale + self.ox
        wy = (ch - sy) / self.scale + self.oy
        self.scale *= factor
        self.ox = wx - sx / self.scale
        self.oy = wy - (ch - sy) / self.scale
        self.redraw()

    def _pan_start(self, event):
        self._pan_last = (event.x, event.y)

    def _pan_move(self, event):
        if self._pan_last is None: return
        dx = event.x - self._pan_last[0]
        dy = event.y - self._pan_last[1]
        self.ox -= dx / self.scale
        self.oy += dy / self.scale
        self._pan_last = (event.x, event.y)
        self.redraw()

    def _pan_end(self, event):
        self._pan_last = None


if __name__ == "__main__":
    root = tk.Tk()
    DxfApp(root)
    root.mainloop()
