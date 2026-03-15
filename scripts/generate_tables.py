#!/usr/bin/env python3
from __future__ import annotations
import argparse
from pathlib import Path
import pandas as pd

def export_table(df, stem, outdir):
    outdir.mkdir(parents=True, exist_ok=True)
    df.to_csv(outdir/f'{stem}.csv', index=False)
    latex = df.to_latex(index=False, float_format=lambda x: f'{x:.4f}')
    (outdir/f'{stem}.tex').write_text(latex)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input-dir', required=True)
    ap.add_argument('--output-dir', required=True)
    args = ap.parse_args()
    indir = Path(args.input_dir)
    outdir = Path(args.output_dir)

    lat = pd.read_csv(indir/'latency_vs_sf.csv')
    lat = lat.groupby(['sf','pacing_strategy'], dropna=False).agg(latency_s_mean=('latency_s_mean','mean'), success_rate=('success_rate','mean')).reset_index()
    export_table(lat, 'table_latency', outdir)

    ene = pd.read_csv(indir/'energy_per_exchange.csv')
    ene = ene.groupby(['sf','ack_batch_size','pacing_strategy'], dropna=False).agg(energy_j_mean=('energy_j_mean','mean'), retransmissions_mean=('retransmissions_mean','mean')).reset_index()
    export_table(ene, 'table_energy', outdir)

    rel = pd.read_csv(indir/'per_vs_load.csv')
    rel = rel.groupby(['campaign','sf','offered_load_eps','pacing_strategy'], dropna=False).agg(success_rate=('success_rate','mean'), per_mean=('per_mean','mean')).reset_index()
    export_table(rel, 'table_reliability', outdir)

if __name__ == '__main__':
    main()
