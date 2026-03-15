#!/usr/bin/env python3
from __future__ import annotations
import argparse, csv, itertools
from pathlib import Path
import yaml

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', required=True)
    ap.add_argument('--campaign', required=True)
    ap.add_argument('--output', required=True)
    args = ap.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text())
    camp = cfg['campaigns'][args.campaign]['selector']
    keys = list(camp.keys())
    values = [camp[k] for k in keys]
    rows = []
    for combo in itertools.product(*values):
        row = dict(zip(keys, combo))
        row.setdefault('on_mean_ms', cfg['sweeps']['on_mean_ms'][0])
        row.setdefault('off_mean_ms', cfg['sweeps']['off_mean_ms'][0])
        row['campaign'] = args.campaign
        row['offered_load_eps'] = row['lambda_total']
        row['pacing_strategy'] = row['pacing']
        row['run'] = int(row['seeds']) if 'seeds' in row else int(row['seed'])
        seed = int(row.pop('seeds', row.get('seed', 1)))
        row['seed'] = seed
        row['run'] = seed
        row['run_id'] = (
            f"{args.campaign}__sf{row['sf']}__g{row['lambda_total']}__p{row['pacing']}"
            f"__ack{row['ack_batch_size']}__rx{row['rx_window_ms']}-{row['rx_idle_ms']}"
            f"__burst{int(bool(row['bursty_enabled']))}__expcol{int(bool(row['use_explicit_event_collisions']))}"
            f"__on{row['on_mean_ms']}__off{row['off_mean_ms']}__seed{seed}"
        ).replace(' ', '')
        rows.append(row)
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

if __name__ == '__main__':
    main()
