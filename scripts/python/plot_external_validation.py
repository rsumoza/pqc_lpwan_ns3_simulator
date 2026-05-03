#!/usr/bin/env python3
import sys
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
from common_plot import ensure_dirs, ieee_axes, FIGURES_DIR, PROCESSED_DIR

ensure_dirs()
ref_dir = Path(sys.argv[1])

rows = []
for csvf in sorted(ref_dir.glob("*.csv")):
    try:
        df = pd.read_csv(csvf)
    except Exception:
        continue
    rows.append({"dataset": csvf.name, "rows": len(df)})

summary = pd.DataFrame(rows)
summary.to_csv(PROCESSED_DIR / "summary_external_validation.csv", index=False)

fig, ax = plt.subplots(figsize=(3.5, 2.2))
ieee_axes(ax)

if len(summary):
    ax.bar(summary["dataset"], summary["rows"])
    ax.set_ylabel("Rows loaded (-)")
    ax.set_xlabel("External validation dataset")
    ax.tick_params(axis="x", rotation=25)
else:
    ax.text(0.5, 0.5, "No external validation CSVs found", ha="center", va="center", transform=ax.transAxes)
    ax.set_axis_off()

fig.tight_layout(pad=0.5)
fig.savefig(FIGURES_DIR / "external_validation_inputs.pdf", bbox_inches="tight")
fig.savefig(FIGURES_DIR / "external_validation_inputs.png", dpi=300, bbox_inches="tight")
plt.close(fig)

if len(summary):
    print(summary.to_string(index=False))
else:
    print("No external validation files found.")
