#!/usr/bin/env python3
from __future__ import annotations
import argparse
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
from plot_style import apply_ieee_style


def savefig(fig, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)


def lineplot(df, x, y, hue, title, xlabel, ylabel, out):
    fig, ax = plt.subplots(figsize=(3.4, 2.3))
    for key, g in df.groupby(hue):
        g = g.sort_values(x)
        ax.plot(g[x], g[y], marker='o', label=str(key))
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.legend(frameon=False)
    savefig(fig, out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input-dir', required=True)
    ap.add_argument('--output-dir', required=True)
    args = ap.parse_args()
    apply_ieee_style()
    indir = Path(args.input_dir)
    outdir = Path(args.output_dir)

    df = pd.read_csv(indir/'per_vs_load.csv')
    lineplot(df[df['campaign'].isin(['baseline','collisions'])], 'offered_load_eps', 'per_mean', 'pacing_strategy', 'PER vs Offered Load', 'Offered load proxy $\lambda_{total}$ (attempts/s)', 'Observed PER', outdir/'per_vs_offered_load.pdf')

    df = pd.read_csv(indir/'latency_vs_sf.csv')
    lineplot(df, 'sf', 'latency_s_mean', 'pacing_strategy', 'Exchange Latency vs SF', 'Spreading factor', 'Latency (s)', outdir/'exchange_latency_vs_sf.pdf')

    df = pd.read_csv(indir/'energy_per_exchange.csv')
    lineplot(df[df['ack_batch_size'].isin([1,4])], 'sf', 'energy_j_mean', 'pacing_strategy', 'Energy per Exchange', 'Spreading factor', 'Energy (J)', outdir/'energy_per_exchange.pdf')

    df = pd.read_csv(indir/'collision_probability_comparison.csv')
    lineplot(df, 'offered_load_eps', 'p_collision_sim_mean', 'collision_model', 'Collision Probability Comparison', 'Offered load proxy $\lambda_{total}$ (attempts/s)', 'Collision probability', outdir/'collision_probability_comparison.pdf')

    df = pd.read_csv(indir/'rx_duty_cycle_impact.csv')
    df['duty_cycle'] = df['rx_window_ms'] / (df['rx_window_ms'] + df['rx_idle_ms'])
    lineplot(df, 'duty_cycle', 'lost_rx_sleep_data_mean', 'pacing_strategy', 'RX Duty Cycle Impact', 'RX duty-cycle availability', 'Sleep-related data losses', outdir/'rx_duty_cycle_impact.pdf')

    df = pd.read_csv(indir/'ack_batching_impact.csv')
    lineplot(df, 'ack_batch_size', 'ack_batch_retx_rounds_mean', 'pacing_strategy', 'ACK Batching Impact', 'ACK batch size', 'ACK recovery rounds', outdir/'ack_batching_impact.pdf')

if __name__ == '__main__':
    main()
