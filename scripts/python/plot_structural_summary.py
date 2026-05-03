#!/usr/bin/env python3
import sys
import numpy as np
import pandas as pd
from common_plot import ensure_dirs, load_df, PROCESSED_DIR

ensure_dirs()
df = load_df(sys.argv[1])

summary = (
    df.groupby("pacing", as_index=False)
      .agg(
          S_ex_mean=("S_ex", "mean"),
          exchange_latency_mean_s=("exchange_latency_s", "mean"),
          n=("S_ex", "count"),
      )
)
summary.to_csv(PROCESSED_DIR / "summary_structural_lockin.csv", index=False)

rows = []
for mode in ["Stochastic", "Phase-bounded"]:
    sub = df[(df["pacing"] == mode) & (df["S_ex"] > 0)]
    if len(sub) == 0:
        continue
    x = sub["exchange_latency_s"].to_numpy()
    rows.append({
        "pacing": mode,
        "n": len(x),
        "mean_s": float(np.mean(x)),
        "std_s": float(np.std(x, ddof=1)),
        "p10_s": float(np.percentile(x, 10)),
        "p25_s": float(np.percentile(x, 25)),
        "median_s": float(np.percentile(x, 50)),
        "p90_s": float(np.percentile(x, 90)),
    })

quant = pd.DataFrame(rows)
quant.to_csv(PROCESSED_DIR / "summary_structural_lockin_quantiles.csv", index=False)

print(summary.to_string(index=False))
print()
print(quant.to_string(index=False))
