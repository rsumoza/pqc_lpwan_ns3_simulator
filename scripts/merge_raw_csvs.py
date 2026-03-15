#!/usr/bin/env python3
from __future__ import annotations
import argparse
from pathlib import Path
import pandas as pd

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input-dir', required=True)
    ap.add_argument('--output', required=True)
    args = ap.parse_args()
    files = [p for p in sorted(Path(args.input_dir).glob('*.csv')) if p.name != 'run_matrix.csv']
    if not files:
        raise SystemExit(f'No raw CSV files found in {args.input_dir}')
    merged = pd.concat([pd.read_csv(p).assign(source_file=p.name) for p in files], ignore_index=True, sort=False)
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    merged.to_csv(args.output, index=False)

if __name__ == '__main__':
    main()
