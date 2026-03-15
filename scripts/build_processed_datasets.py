#!/usr/bin/env python3
from __future__ import annotations
import argparse
from pathlib import Path
import pandas as pd
import numpy as np

def agg(df, group_cols):
    return df.groupby(group_cols, dropna=False).agg(
        n=('run_id','count'),
        success_rate=('success','mean'),
        per_mean=('per_observed','mean'),
        per_std=('per_observed','std'),
        latency_s_mean=('exchange_latency_s','mean'),
        latency_s_std=('exchange_latency_s','std'),
        energy_j_mean=('energy_j_exchange','mean'),
        energy_j_std=('energy_j_exchange','std'),
        retransmissions_mean=('retransmissions_count','mean'),
        airtime_ms_mean=('tx_airtime_ms_total','mean'),
        p_collision_sim_mean=('per_collision_sim_avg','mean'),
        p_collision_th_mean=('per_collision_th_avg','mean'),
        p_phy_mean=('per_channel_avg','mean'),
        p_total_mean=('per_total_avg','mean'),
        lost_rx_sleep_data_mean=('lost_rx_sleep_data','mean'),
        lost_ack_sleep_mean=('lost_ack_sleep','mean'),
        lost_ack_channel_mean=('lost_ack_channel','mean'),
        ack_batch_retx_rounds_mean=('ack_batch_retx_rounds','mean'),
        ack_frames_tx_mean=('ack_frames_tx','mean'),
    ).reset_index()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input-dir', required=True)
    ap.add_argument('--output-dir', required=True)
    args = ap.parse_args()
    indir = Path(args.input_dir)
    outdir = Path(args.output_dir)
    outdir.mkdir(parents=True, exist_ok=True)
    df = pd.concat([pd.read_csv(p) for p in sorted(indir.glob('*.csv'))], ignore_index=True, sort=False)
    df.to_csv(outdir/'all_exchanges.csv', index=False)

    agg(df, ['campaign','sf','offered_load_eps','pacing_strategy']).to_csv(outdir/'per_vs_load.csv', index=False)
    agg(df, ['campaign','sf','pacing_strategy']).to_csv(outdir/'latency_vs_sf.csv', index=False)
    agg(df, ['campaign','sf','pacing_strategy','ack_batch_size']).to_csv(outdir/'energy_per_exchange.csv', index=False)
    agg(df, ['campaign','sf','rx_window_ms','rx_idle_ms','pacing_strategy']).to_csv(outdir/'rx_duty_cycle_impact.csv', index=False)
    agg(df, ['campaign','sf','ack_batch_size','rx_idle_ms','pacing_strategy']).to_csv(outdir/'ack_batching_impact.csv', index=False)
    agg(df, ['campaign','sf','offered_load_eps','ack_batch_size','rx_window_ms','rx_idle_ms','pacing_strategy']).to_csv(outdir/'summary_by_campaign.csv', index=False)

    comp = df.groupby(['campaign','sf','offered_load_eps','pacing_strategy','use_explicit_event_collisions'], dropna=False).agg(
        n=('run_id','count'),
        measured_per=('per_observed','mean'),
        p_collision_sim_mean=('per_collision_sim_avg','mean'),
        p_collision_th_mean=('per_collision_th_avg','mean'),
        p_phy_mean=('per_channel_avg','mean'),
        p_total_formula=('p_total','mean'),
    ).reset_index()
    comp['collision_model'] = np.where(comp['use_explicit_event_collisions'] > 0, 'explicit', 'analytical')
    comp.to_csv(outdir/'collision_probability_comparison.csv', index=False)

if __name__ == '__main__':
    main()
