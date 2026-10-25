#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DXF Auto-Close Contours — портативный GUI-инструмент.
Находит контуры из связанных LINE/ARC/POLYLINE/SPLINE и замыкает незамкнутые.

Зависимости:  pip install ezdxf
Сборка exe:   pyinstaller --onefile --windowed dxf_autoclose.py
"""

import os
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext
from math import floor

import ezdxf
from ezdxf.math import Vec2


def vec2(p):
    return Vec2(p[0], p[1])


def get_endpoints(e):
    t = e.dxftype()
    if t == 'LINE':
        return vec2(e.dxf.start), vec2(e.dxf.end)
    if t == 'ARC':
        return vec2(e.start_point), vec2(e.end_point)
    if t == 'LWPOLYLINE':
        if e.closed:
            return None
        pts = list(e.get_points('xy'))
        if len(pts) < 2:
            return None
        return Vec2(*pts[0]), Vec2(*pts[-1])
    if t == 'POLYLINE':
        if e.is_closed:
            return None
        pts = [vec2(v.dxf.location) for v in e.vertices]
        if len(pts) < 2:
            return None
        return pts[0], pts[-1]
    if t == 'SPLINE':
        try:
            return vec2(e.start_point), vec2(e.end_point)
        except Exception:
            return None
    return None


def process_dxf(in_path, out_path, tol, aggressive, log):
    log(f"Читаю: {in_path}")
    doc = ezdxf.readfile(in_path)
    msp = doc.modelspace()

    if "AUTOCLOSED" not in doc.layers:
        doc.layers.add("AUTOCLOSED", color=1)  # красный

    # ---------- индексация узлов (сетка + допуск) ----------
    nodes = []
    buckets = {}
    cell = tol * 2 if tol > 0 else 1e-6

    def node_id(pt):
        key = (floor(pt.x / cell), floor(pt.y / cell))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                k = (key[0] + dx, key[1] + dy)
                for i in buckets.get(k, ()):
                    if (nodes[i] - pt).magnitude < tol:
                        return i
        nodes.append(pt)
        buckets.setdefault(key, []).append(len(nodes) - 1)
        return len(nodes) - 1

    # ---------- сбор рёбер ----------
    edges = []
    already_closed = 0
    for e in msp:
        t = e.dxftype()
        if t == 'CIRCLE':
            already_closed += 1
            continue
        if t in ('LWPOLYLINE', 'POLYLINE'):
            try:
                if (t == 'LWPOLYLINE' and e.closed) or (t == 'POLYLINE' and e.is_closed):
                    already_closed += 1
                    continue
            except Exception:
                pass
        try:
            ep = get_endpoints(e)
        except Exception as ex:
            log(f"  ! пропуск {t}: {ex}")
            continue
        if ep is None:
            continue
        a = node_id(ep[0])
        b = node_id(ep[1])
        if a == b:
            already_closed += 1
            continue
        edges.append((a, b))

    # ---------- union-find ----------
    parent = list(range(len(nodes)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for a, b in edges:
        union(a, b)

    comps = {}
    for i in range(len(nodes)):
        comps.setdefault(find(i), []).append(i)

    deg = [0] * len(nodes)
    for a, b in edges:
        deg[a] += 1
        deg[b] += 1

    # ---------- анализ контуров ----------
    closed_found = 0
    closed_auto = 0
    failed = 0
    added = 0

    def add_line(p1, p2):
        nonlocal added
        ln = msp.add_line((p1.x, p1.y, 0), (p2.x, p2.y, 0))
        ln.dxf.layer = "AUTOCLOSED"
        added += 1

    for _root, members in comps.items():
        if len(members) == 1 and deg[members[0]] == 0:
            continue
        open_nodes = [n for n in members if deg[n] == 1]

        if not open_nodes:
            closed_found += 1
            continue

        if len(open_nodes) == 2:
            p1, p2 = nodes[open_nodes[0]], nodes[open_nodes[1]]
            add_line(p1, p2)
            log(f"  + замкнут разрыв {p1} -> {p2}  (d={((p1-p2).magnitude):.4f})")
            closed_auto += 1
            continue

        if aggressive:
            to_pair = list(open_nodes)
            while len(to_pair) >= 2:
                best = None
                for i in range(len(to_pair)):
                    for j in range(i + 1, len(to_pair)):
                        d = (nodes[to_pair[i]] - nodes[to_pair[j]]).magnitude
                        if best is None or d < best[0]:
                            best = (d, i, j)
                d, i, j = best
                p1, p2 = nodes[to_pair[i]], nodes[to_pair[j]]
                add_line(p1, p2)
                log(f"  + замкнут разрыв {p1} -> {p2}  (d={d:.4f})")
                closed_auto += 1
                for k in sorted([i, j], reverse=True):
                    to_pair.pop(k)
            if len(to_pair) == 1:
                log("  ! остался один свободный конец — не замыкается")
                failed += 1
        else:
            log(f"  ! в компоненте {len(open_nodes)} свободных концов — пропущено (включите Aggressive)")
            failed += 1

    log("")
    log("=== Итог ===")
    log(f"Изначально замкнутых (окружности/closed polylines): {already_closed}")
    log(f"Найдено замкнутых контуров из кусков:            {closed_found}")
    log(f"Автоматически замкнуто контуров:                  {closed_auto}")
    log(f"Добавлено замыкающих линий:                       {added}")
    log(f"Компонент пропущено:                              {failed}")
    log(f"Сохраняю: {out_path}")
    doc.saveas(out_path)
    log("Готово.")


class App:
    def __init__(self, root):
        self.root = root
        root.title("DXF Auto-Close Contours")
        root.geometry("780x560")

        f = tk.Frame(root)
        f.pack(fill='x', padx=10, pady=8)
        f.columnconfigure(1, weight=1)

        tk.Label(f, text="DXF-файл:").grid(row=0, column=0, sticky='w')
        self.path_var = tk.StringVar()
        tk.Entry(f, textvariable=self.path_var).grid(row=0, column=1, sticky='we', padx=4)
        tk.Button(f, text="Обзор...", command=self.browse).grid(row=0, column=2)

        tk.Label(f, text="Допуск:").grid(row=1, column=0, sticky='w')
        self.tol_var = tk.StringVar(value="0.001")
        tk.Entry(f, textvariable=self.tol_var, width=15).grid(row=1, column=1, sticky='w', padx=4)

        self.aggr_var = tk.BooleanVar(value=False)
        tk.Checkbutton(f, text="Aggressive: соединять ближайшие концы, если их >2",
                       variable=self.aggr_var).grid(row=2, column=0, columnspan=3, sticky='w')

        self.overwrite_var = tk.BooleanVar(value=False)
        tk.Checkbutton(f, text="Перезаписать исходник (иначе — <имя>_closed.dxf)",
                       variable=self.overwrite_var).grid(row=3, column=0, columnspan=3, sticky='w')

        tk.Button(root, text="Обработать", height=2,
                  command=self.process).pack(fill='x', padx=10)

        self.logw = scrolledtext.ScrolledText(root, height=20)
        self.logw.pack(fill='both', expand=True, padx=10, pady=10)

    def log(self, msg):
        self.logw.insert('end', msg + '\n')
        self.logw.see('end')
        self.root.update_idletasks()

    def browse(self):
        p = filedialog.askopenfilename(filetypes=[("DXF files", "*.dxf")])
        if p:
            self.path_var.set(p)

    def process(self):
        self.logw.delete('1.0', 'end')
        path = self.path_var.get().strip()
        if not path or not os.path.isfile(path):
            messagebox.showerror("Ошибка", "Выберите существующий DXF-файл.")
            return
        try:
            tol = float(self.tol_var.get())
            if tol <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Ошибка", "Допуск должен быть положительным числом.")
            return
        try:
            out = path if self.overwrite_var.get() else \
                os.path.splitext(path)[0] + "_closed.dxf"
            self.log(f"Допуск: {tol}")
            self.log(f"Aggressive: {self.aggr_var.get()}")
            self.log("")
            process_dxf(path, out, tol, self.aggr_var.get(), self.log)
            messagebox.showinfo("Готово", f"Сохранено:\n{out}")
        except Exception as ex:
            messagebox.showerror("Ошибка", str(ex))
            self.log(f"ERROR: {ex}")


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()