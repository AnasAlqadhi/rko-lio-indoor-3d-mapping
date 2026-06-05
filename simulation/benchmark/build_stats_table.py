#!/usr/bin/env python3
"""
Collect all evo APE/RPE stats (mean/median/std/min/max/rmse) for every
method x world into a tidy CSV and a paper-ready LaTeX table.

Reads:  results/{liosam,rkolio,fastlio}_{ape,rpe}_{small_house,warehouse,bookstore}.txt
Writes: results/comparison_stats.csv
        results/comparison_table.tex
"""

import re
from pathlib import Path

OUT = Path.home() / "simulation_experiment/results"
METHODS = ["liosam", "rkolio", "fastlio"]
METHOD_LABEL = {"liosam": "LIO-SAM", "rkolio": "RKO-LIO", "fastlio": "FAST-LIO"}
WORLDS = ["small_house", "warehouse", "bookstore"]
WORLD_LABEL = {"small_house": "Small House", "warehouse": "Warehouse", "bookstore": "Bookstore"}
METRICS = ["ape", "rpe"]
STATS = ["rmse", "mean", "median", "std", "min", "max"]


def parse(path: Path):
    d = {}
    if not path.exists():
        return d
    for line in path.read_text().splitlines():
        m = re.match(r"\s*(rmse|mean|median|std|min|max)\s+([0-9.eE+-]+)", line)
        if m:
            d[m.group(1)] = float(m.group(2))
    return d


# ── CSV ──────────────────────────────────────────────────────────────────────
rows = [["world", "method", "metric"] + STATS]
data = {}
for w in WORLDS:
    for meth in METHODS:
        for metric in METRICS:
            stats = parse(OUT / f"{meth}_{metric}_{w}.txt")
            data[(w, meth, metric)] = stats
            rows.append([w, meth, metric] + [f"{stats.get(s, ''):.6f}" if s in stats else "" for s in STATS])

csv_path = OUT / "comparison_stats.csv"
csv_path.write_text("\n".join(",".join(map(str, r)) for r in rows) + "\n")
print(f"Wrote {csv_path}")

# ── LaTeX (APE & RPE RMSE, bold best per world) ───────────────────────────────
def best_method(w, metric):
    vals = {m: data[(w, m, metric)].get("rmse") for m in METHODS}
    vals = {m: v for m, v in vals.items() if v is not None}
    return min(vals, key=vals.get) if vals else None

tex = []
tex.append(r"\begin{table}[t]")
tex.append(r"\centering")
tex.append(r"\caption{Trajectory accuracy (RMSE, m) on three simulated environments. "
           r"APE: absolute pose error; RPE: relative pose error over 1\,m. Best per row in bold.}")
tex.append(r"\label{tab:sim_comparison}")
tex.append(r"\begin{tabular}{llccc}")
tex.append(r"\toprule")
tex.append(r"Environment & Metric & LIO-SAM & RKO-LIO & FAST-LIO \\")
tex.append(r"\midrule")
for w in WORLDS:
    for metric in METRICS:
        best = best_method(w, metric)
        cells = []
        for meth in METHODS:
            v = data[(w, meth, metric)].get("rmse")
            s = f"{v:.3f}" if v is not None else "--"
            if meth == best:
                s = r"\textbf{" + s + "}"
            cells.append(s)
        env = WORLD_LABEL[w] if metric == "ape" else ""
        tex.append(f"{env} & {metric.upper()} & " + " & ".join(cells) + r" \\")
    tex.append(r"\midrule")
tex[-1] = r"\bottomrule"
tex.append(r"\end{tabular}")
tex.append(r"\end{table}")

tex_path = OUT / "comparison_table.tex"
tex_path.write_text("\n".join(tex) + "\n")
print(f"Wrote {tex_path}")

# ── console preview ───────────────────────────────────────────────────────────
print("\n=== APE / RPE RMSE (m) ===")
print(f"{'world':12} {'metric':6} {'LIO-SAM':>10} {'RKO-LIO':>10} {'FAST-LIO':>10}")
for w in WORLDS:
    for metric in METRICS:
        vals = [data[(w, m, metric)].get("rmse") for m in METHODS]
        cells = [f"{v:10.3f}" if v is not None else f"{'--':>10}" for v in vals]
        print(f"{w:12} {metric.upper():6} " + " ".join(cells))
