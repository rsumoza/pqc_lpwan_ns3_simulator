#!/usr/bin/env python3
import sys
import numpy as np
import matplotlib.pyplot as plt
from common_plot import ensure_dirs, ieee_axes, load_df, lambda_axis_label, FIGURES_DIR, PROCESSED_DIR

ensure_dirs()
df = load_df(sys.argv[1])

rows = []
for (lam, pacing), sub in df.groupby(["lambda_eff", "pacing"]):
    succ = sub[sub["S_ex"] > 0]
    rows.append({
        "lambda_eff": lam,
        "pacing": pacing,
        "E_succ": np.nan if len(succ) == 0 else succ["energy_j_exchange"].mean(),
        "S_ex_mean": sub["S_ex"].mean(),
        "n": len(sub),
        "n_success": len(succ),
    })

summary = __import__("pandas").DataFrame(rows).sort_values(["pacing", "lambda_eff"])
summary.to_csv(PROCESSED_DIR / "summary_energy_per_success_main.csv", index=False)

fig, ax = plt.subplots(figsize=(3.5, 2.35))
ieee_axes(ax)

for mode, marker in [("Stochastic", "s"), ("Phase-bounded", "^")]:
    sub = summary[summary["pacing"] == mode]
    if len(sub):
        ax.plot(sub["lambda_eff"], sub["E_succ"], marker=marker, linewidth=1.1, label=mode)

ax.set_xlabel(lambda_axis_label())
ax.set_ylabel(r"Energy per successful exchange $E_{\mathrm{succ}}$ (J)")
ax.set_xlim(0.45, 2.05)
ax.set_xticks([0.5, 1.0, 2.0])
ax.legend(frameon=False, loc="upper right", prop={"size": 6})

fig.tight_layout(pad=0.5)
fig.savefig(FIGURES_DIR / "energy_per_success_three_pacing.pdf", bbox_inches="tight")
fig.savefig(FIGURES_DIR / "energy_per_success_three_pacing.png", dpi=300, bbox_inches="tight")
plt.close(fig)

print(summary.to_string(index=False))
