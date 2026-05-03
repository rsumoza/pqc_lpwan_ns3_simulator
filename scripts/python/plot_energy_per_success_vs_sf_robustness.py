#!/usr/bin/env python3
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from common_plot import ensure_dirs, ieee_axes, load_df, lambda_panel_title, FIGURES_DIR, PROCESSED_DIR

ensure_dirs()
df = load_df(sys.argv[1])

rows = []
for (lam, sf, pacing), sub in df.groupby(["lambda_eff", "sf", "pacing"]):
    succ = sub[sub["S_ex"] > 0]
    rows.append({
        "lambda_eff": lam,
        "sf": sf,
        "pacing": pacing,
        "E_succ": np.nan if len(succ) == 0 else succ["energy_j_exchange"].mean(),
        "S_ex_mean": sub["S_ex"].mean(),
        "n": len(sub),
        "n_success": len(succ),
    })

summary = pd.DataFrame(rows).sort_values(["lambda_eff", "pacing", "sf"])
summary.to_csv(PROCESSED_DIR / "summary_energy_per_success_vs_sf_robustness.csv", index=False)

for lam in sorted(summary["lambda_eff"].unique()):
    sub_lam = summary[summary["lambda_eff"] == lam].copy()
    fig, ax = plt.subplots(figsize=(3.5, 2.35))
    ieee_axes(ax)

    for mode, marker in [("Stochastic", "s"), ("Phase-bounded", "^")]:
        sub = sub_lam[sub_lam["pacing"] == mode]
        if len(sub):
            ax.plot(sub["sf"], sub["E_succ"], marker=marker, linewidth=1.1, label=mode)

    ax.set_title(lambda_panel_title(lam), pad=2.0)
    ax.set_xlabel("Spreading factor (SF)")
    ax.set_ylabel(r"Energy per successful exchange $E_{\mathrm{succ}}$ (J)")
    ax.set_xticks(sorted(sub_lam["sf"].unique()))
    ax.legend(frameon=False, loc="upper left", prop={"size": 6})

    fig.tight_layout(pad=0.5)
    fig.savefig(FIGURES_DIR / f"energy_per_success_vs_sf_lambda_{int(lam)}.pdf", bbox_inches="tight")
    fig.savefig(FIGURES_DIR / f"energy_per_success_vs_sf_lambda_{int(lam)}.png", dpi=300, bbox_inches="tight")
    plt.close(fig)

print(summary.to_string(index=False))
