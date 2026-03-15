#!/usr/bin/env python3
from __future__ import annotations
import argparse
from pathlib import Path
import pandas as pd

def nearest_merge(sim, ref):
    rows = []
    for _, r in sim.iterrows():
        sub = ref[ref['sf'] == r['sf']].copy()
        if sub.empty:
            continue
        sub['dist'] = (sub['snr_db'] - r['snr_db']).abs()
        m = sub.sort_values('dist').iloc[0]
        rows.append({'sf': r['sf'], 'snr_db': r['snr_db'], 'sim_p_phy': r['per_channel_avg'], 'ref_per': m['per_ref'], 'abs_err': abs(r['per_channel_avg'] - m['per_ref'])})
    return pd.DataFrame(rows)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input', required=True)
    ap.add_argument('--reference', required=True)
    ap.add_argument('--output', required=True)
    args = ap.parse_args()
    sim = pd.read_csv(args.input)[['sf','snr_db','per_channel_avg']].dropna().drop_duplicates()
    ref = pd.read_csv(args.reference)
    merged = nearest_merge(sim, ref)
    summary = merged.groupby('sf', dropna=False).agg(mean_abs_err=('abs_err','mean'), n=('abs_err','count')).reset_index()
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    summary.to_csv(args.output, index=False)

if __name__ == '__main__':
    main()
