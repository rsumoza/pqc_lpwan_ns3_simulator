#!/usr/bin/env python3
from __future__ import annotations
import argparse
from pathlib import Path
import numpy as np
import pandas as pd

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input', required=True)
    ap.add_argument('--output', required=True)
    args = ap.parse_args()
    df = pd.read_csv(args.input)
    airtime_s = pd.to_numeric(df['airtime_ms_frag_avg'], errors='coerce') / 1000.0
    lam = pd.to_numeric(df['lambda_total'], errors='coerce')
    channels = pd.to_numeric(df.get('logical_channels', 64), errors='coerce')
    g = lam * airtime_s / channels
    df['p_collision_aloha_theory'] = 1.0 - np.exp(-2.0 * g)
    out = df.groupby(['campaign','sf','offered_load_eps','pacing_strategy'], dropna=False).agg(
        p_collision_sim_mean=('per_collision_sim_avg','mean'),
        p_collision_th_mean=('per_collision_th_avg','mean'),
        p_collision_aloha_theory=('p_collision_aloha_theory','mean'),
    ).reset_index()
    out['abs_err_vs_sim'] = (out['p_collision_sim_mean'] - out['p_collision_aloha_theory']).abs()
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    out.to_csv(args.output, index=False)

if __name__ == '__main__':
    main()
