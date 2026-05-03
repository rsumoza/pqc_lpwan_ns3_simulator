#!/usr/bin/env python3
import sys
import pandas as pd
import matplotlib.pyplot as plt
from common_plot import ensure_dirs, ieee_axes, load_df, FIGURES_DIR, PROCESSED_DIR

ECW_VALUES = [3.0, 5.0, 8.0]
PACING_ORDER = ["Baseline", "Stochastic", "Phase-bounded"]
MARKERS = {"Baseline": "o", "Stochastic": "s", "Phase-bounded": "^"}

ensure_dirs()
df = load_df(sys.argv[1])

if "exchange_latency_s" not in df.columns:
    raise ValueError("Missing required column: exchange_latency_s")

df["completed"] = df["S_ex"] > 0

lambda_col = "lambda_eff" if "lambda_eff" in df.columns else None
if lambda_col is not None and 1.0 in set(df[lambda_col].dropna().astype(float)):
    df_plot = df[df[lambda_col].astype(float) == 1.0].copy()
    lambda_scope = "lambda=1"
else:
    df_plot = df.copy()
    lambda_scope = "all"

rows = []
for ecw in ECW_VALUES:
    tmp = df_plot.copy()
    tmp["success_ecw"] = tmp["completed"] & (tmp["exchange_latency_s"] <= ecw)

    for pacing in PACING_ORDER:
        sub = tmp[tmp["pacing"] == pacing]
        if len(sub):
            rows.append({
                "T_ECW_s": ecw,
                "pacing": pacing,
                "S_ex": sub["success_ecw"].mean(),
                "n": len(sub),
                "lambda_scope": lambda_scope,
            })

summary = pd.DataFrame(rows)
summary.to_csv(PROCESSED_DIR / "summary_success_vs_ecw.csv", index=False)

fig, ax = plt.subplots(figsize=(3.5, 2.35))
ieee_axes(ax)

for pacing in PACING_ORDER:
    sub = summary[summary["pacing"] == pacing].sort_values("T_ECW_s")
    if len(sub):
        ax.plot(sub["T_ECW_s"], sub["S_ex"], marker=MARKERS[pacing], linewidth=1.1, markersize=3.2, label=pacing)

ax.set_xlabel(r"Exchange Completion Window $T_{\mathrm{ECW}}$ (s)")
ax.set_ylabel(r"Exchange success probability $S_{\mathrm{ex}}$ (-)")
ax.set_xticks(ECW_VALUES)
ax.set_ylim(-0.02, 1.02)
ax.legend(frameon=False, loc="lower right", prop={"size": 6})

fig.tight_layout(pad=0.5)
fig.savefig(FIGURES_DIR / "success_vs_ecw.pdf", bbox_inches="tight")
fig.savefig(FIGURES_DIR / "success_vs_ecw.png", dpi=300, bbox_inches="tight")
plt.close(fig)

print(summary.to_string(index=False))
print("[OK] wrote figures/success_vs_ecw.pdf")
