#!/usr/bin/env python3
from __future__ import annotations
import argparse, csv, os, shlex, subprocess, sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import pandas as pd
import yaml

def _format_bool(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    s = str(v).strip().lower()
    return 'true' if s in {'1','true','yes','y'} else 'false'

def build_command(row: dict[str, str], cfg: dict, sim_bin: str, out_csv: Path) -> list[str]:
    base = cfg['simulator']['base_args']
    cmd = [sim_bin]
    def add(k, v):
        cmd.append(f'--{k}={v}')
    for k, v in base.items():
        add(k, v)
    add('outCsv', str(out_csv))
    add('sf', row['sf'])
    add('pacing', row['pacing'])
    add('lambdaTotal', row['lambda_total'])
    add('ackBatchSize', row['ack_batch_size'])
    add('rxWindowMs', row['rx_window_ms'])
    add('rxIdleMs', row['rx_idle_ms'])
    add('rxDutyCycleEnabled', _format_bool(float(row['rx_idle_ms']) > 0.0))
    add('burstyEnabled', _format_bool(row['bursty_enabled']))
    add('onMeanMs', row['on_mean_ms'])
    add('offMeanMs', row['off_mean_ms'])
    add('useExplicitEventCollisions', _format_bool(row['use_explicit_event_collisions']))
    add('variant', row['campaign'])
    add('seed', row['seed'])
    add('run', row['run'])
    return cmd

def finalize_csv(out_csv: Path, row: dict[str, str]):
    df = pd.read_csv(out_csv)
    if len(df) != 1:
        raise RuntimeError(f'{out_csv} expected exactly 1 ExchangeResult row, got {len(df)}')
    add_cols = {
        'campaign': row['campaign'],
        'run_id': row['run_id'],
        'seed': int(row['seed']),
        'run': int(row['run']),
        'lambda_total': float(row['lambda_total']),
        'logical_channels': 64,
        'bursty_enabled': int(str(row['bursty_enabled']).lower() in {'true','1'}),
        'on_mean_ms': float(row['on_mean_ms']),
        'off_mean_ms': float(row['off_mean_ms']),
        'use_explicit_event_collisions': int(str(row['use_explicit_event_collisions']).lower() in {'true','1'}),
        'offered_load_eps': float(row['offered_load_eps']),
        'pacing_strategy': row['pacing_strategy'],
    }
    for k, v in add_cols.items():
        df[k] = v
    df.to_csv(out_csv, index=False)

def run_one(row, cfg, sim_bin, sim_runner, raw_dir):
    out_csv = Path(raw_dir) / f"{row['run_id']}.csv"
    ok = out_csv.with_suffix('.ok')
    if out_csv.exists() and ok.exists():
        return str(out_csv)
    cmd = build_command(row, cfg, sim_bin, out_csv)
    if sim_runner:
        # shell wrapper mode, e.g. SIM_RUNNER='./ns3 run'
        payload = ' '.join(shlex.quote(c) for c in cmd)
        full_cmd = f"{sim_runner} {shlex.quote(payload)}"
        proc = subprocess.run(full_cmd, shell=True, capture_output=True, text=True)
    else:
        proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"Run failed for {row['run_id']}\nSTDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}")
    finalize_csv(out_csv, row)
    ok.write_text('ok\n')
    return str(out_csv)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', required=True)
    ap.add_argument('--campaign', required=True)
    ap.add_argument('--matrix', required=True)
    ap.add_argument('--sim-bin', required=True)
    ap.add_argument('--sim-runner', default='')
    ap.add_argument('--raw-dir', required=True)
    ap.add_argument('--jobs', type=int, default=4)
    args = ap.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text())
    rows = list(csv.DictReader(open(args.matrix, newline='')))
    Path(args.raw_dir).mkdir(parents=True, exist_ok=True)
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as ex:
        futs = [ex.submit(run_one, row, cfg, args.sim_bin, args.sim_runner, args.raw_dir) for row in rows]
        for fut in as_completed(futs):
            print(fut.result())

if __name__ == '__main__':
    main()
