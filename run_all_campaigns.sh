#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$ROOT_DIR/campaigns/common.sh"
for c in baseline bursty collisions rxsleep ackbatch; do
  echo "=== Running campaign: $c ==="
  run_campaign "$c"
done
python3 "$ROOT_DIR/scripts/clean_raw_csv.py" --inputs "$ROOT_DIR"/datasets/merged/*.csv --output-dir "$ROOT_DIR/datasets/processed/cleaned" --config "$ROOT_DIR/config/experiment_matrix.yaml"
python3 "$ROOT_DIR/scripts/build_processed_datasets.py" --input-dir "$ROOT_DIR/datasets/processed/cleaned" --output-dir "$ROOT_DIR/datasets/processed"
python3 "$ROOT_DIR/figures/scripts/generate_all_figures.py" --input-dir "$ROOT_DIR/datasets/processed" --output-dir "$ROOT_DIR/figures/output"
python3 "$ROOT_DIR/scripts/generate_tables.py" --input-dir "$ROOT_DIR/datasets/processed" --output-dir "$ROOT_DIR/tables/paper_tables"
python3 "$ROOT_DIR/validation/theory/aloha_validation.py" --input "$ROOT_DIR/datasets/processed/all_exchanges.csv" --output "$ROOT_DIR/validation/theory/aloha_validation_summary.csv"
python3 "$ROOT_DIR/validation/theory/per_snr_validation.py" --input "$ROOT_DIR/datasets/processed/all_exchanges.csv" --reference "$ROOT_DIR/validation/references/per_vs_snr_reference.csv" --output "$ROOT_DIR/validation/theory/per_snr_validation_summary.csv"
echo "Artifact pipeline completed successfully."
