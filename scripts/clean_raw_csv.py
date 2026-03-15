#!/usr/bin/env python3
from __future__ import annotations
import argparse
from pathlib import Path
import pandas as pd
import yaml

BOOLEAN_COLUMNS = ['validation_mode','rx_duty_cycle_enabled','per_exchange_fail']

ARTIFACT_REQUIRED = [
    'campaign','run_id','seed','run','sf','lambda_total','offered_load_eps','pacing_strategy','rx_window_ms','rx_idle_ms',
    'ack_batch_size','ok_frags','fail_frags','per_observed','tx_airtime_ms_total','exchange_latency_s','energy_j_exchange',
    'retransmissions_count','lost_rx_sleep_data','lost_ack_sleep','lost_ack_channel','frames_tx_total','data_frames_tx','ack_frames_tx',
    'per_collision_sim_avg','per_collision_th_avg','per_channel_avg','per_total_avg','pacing_mode','use_explicit_event_collisions'
]

def clean(df: pd.DataFrame) -> pd.DataFrame:
    for c in df.columns:
        if df[c].dtype == object:
            df[c] = df[c].astype(str).str.strip()
    for c in BOOLEAN_COLUMNS:
        if c in df.columns:
            df[c] = df[c].astype(str).str.lower().map({'true':1,'false':0}).fillna(df[c]).astype(int)
    numeric_candidates = [c for c in df.columns if c not in {'environment','variant','pacing_mode','campaign','run_id','source_file','pacing_strategy'}]
    for c in numeric_candidates:
        df[c] = pd.to_numeric(df[c], errors='ignore')
    if 'success' not in df.columns:
        df['success'] = ((df['per_exchange_fail'] == 0) & (df['ok_frags'] == df['n_fragments_exchange'])).astype(int)
    if 'exchange_latency_ms' not in df.columns:
        df['exchange_latency_ms'] = 1000.0 * pd.to_numeric(df['exchange_latency_s'], errors='coerce')
    if 'p_phy' not in df.columns:
        df['p_phy'] = pd.to_numeric(df['per_channel_avg'], errors='coerce')
    if 'p_collision' not in df.columns:
        df['p_collision'] = pd.to_numeric(df['per_collision_sim_avg'], errors='coerce')
    if 'p_total' not in df.columns:
        df['p_total'] = 1.0 - (1.0 - df['p_phy'].clip(0, 1)) * (1.0 - df['p_collision'].clip(0, 1))
    return df.drop_duplicates()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--inputs', nargs='+', required=True)
    ap.add_argument('--output-dir', required=True)
    ap.add_argument('--config', default=None)
    args = ap.parse_args()
    outdir = Path(args.output_dir)
    outdir.mkdir(parents=True, exist_ok=True)
    required = ARTIFACT_REQUIRED
    if args.config:
        cfg = yaml.safe_load(Path(args.config).read_text())
        required = cfg['csv_contract']['stable_header'] + cfg['csv_contract']['artifact_added_columns']
    for fp in args.inputs:
        path = Path(fp)
        df = pd.read_csv(path)
        missing = [c for c in required if c not in df.columns]
        if missing:
            raise SystemExit(f'{path}: missing required columns: {missing}')
        clean(df).to_csv(outdir / path.name, index=False)

if __name__ == '__main__':
    main()
