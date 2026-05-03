#!/usr/bin/env python3
import sys
import matplotlib.pyplot as plt
from common_plot import ensure_dirs, ieee_axes, load_df, lambda_axis_label, FIGURES_DIR, PROCESSED_DIR

ensure_dirs()
df = load_df(sys.argv[1])

summary = (
    df.groupby(["lambda_eff", "pacing"], as_index=False)
      .agg(S_ex_mean=("S_ex", "mean"), n=("S_ex", "count"))
      .sort_values(["pacing", "lambda_eff"])
)
summary.to_csv(PROCESSED_DIR / "summary_success_vs_lambda_main.csv", index=False)

fig, ax = plt.subplots(figsize=(3.5, 2.35))
ieee_axes(ax)

for mode, marker in [("Baseline", "o"), ("Stochastic", "s"), ("Phase-bounded", "^")]:
    sub = summary[summary["pacing"] == mode]
    if len(sub):
        ax.plot(sub["lambda_eff"], sub["S_ex_mean"], marker=marker, linewidth=1.1, label=mode)

ax.set_xlabel(lambda_axis_label())
ax.set_ylabel(r"Exchange success probability $S_{\mathrm{ex}}$ (-)")
ax.set_xlim(-0.05, 2.05)
ax.set_xticks([0, 0.5, 1.0, 2.0])
ax.set_ylim(-0.02, 1.02)
ax.legend(frameon=False, loc="lower right", prop={"size": 6})

fig.tight_layout(pad=0.5)
fig.savefig(FIGURES_DIR / "success_vs_lambda_three_pacing.pdf", bbox_inches="tight")
fig.savefig(FIGURES_DIR / "success_vs_lambda_three_pacing.png", dpi=300, bbox_inches="tight")
plt.close(fig)

print(summary.to_string(index=False))
