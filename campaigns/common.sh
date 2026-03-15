#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="$ROOT_DIR/config/experiment_matrix.yaml"
SIM_BIN="${SIM_BIN:-./build/scratch/ns3.47-lpwan-pqc-export-exchanges-default}"
SIM_RUNNER="${SIM_RUNNER:-}"
JOBS="${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)}"
export PYTHONUNBUFFERED=1
mkdir -p "$ROOT_DIR/datasets/raw" "$ROOT_DIR/datasets/merged" "$ROOT_DIR/datasets/processed/cleaned" "$ROOT_DIR/figures/output" "$ROOT_DIR/tables/paper_tables"
run_campaign() {
  local campaign="$1"
  local raw_dir="$ROOT_DIR/datasets/raw/$campaign"
  local matrix_csv="$raw_dir/run_matrix.csv"
  mkdir -p "$raw_dir"
  python3 "$ROOT_DIR/scripts/generate_run_matrix.py"     --config "$CONFIG_FILE"     --campaign "$campaign"     --output "$matrix_csv"
  python3 "$ROOT_DIR/scripts/run_campaign.py"     --config "$CONFIG_FILE"     --campaign "$campaign"     --matrix "$matrix_csv"     --sim-bin "$SIM_BIN"     --sim-runner "$SIM_RUNNER"     --raw-dir "$raw_dir"     --jobs "$JOBS"
  python3 "$ROOT_DIR/scripts/merge_raw_csvs.py"     --input-dir "$raw_dir"     --output "$ROOT_DIR/datasets/merged/${campaign}.csv"
}
