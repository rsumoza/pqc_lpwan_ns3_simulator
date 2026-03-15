# Reproducible Artifact for the Frozen PQC LoRaWAN Exchange Simulator

This repository is a **drop-in reproducibility layer** around the frozen ns-3 simulator and scratch driver `lpwan-pqc-export-exchanges.cc`. It does **not** modify simulator code. Its purpose is to let another researcher reproduce experiments, regenerate datasets, rebuild figures, and recreate paper tables from the stable `ExchangeResult` CSV contract.

## Repository structure

```text
repo/
├── simulator/
│   └── ns3-module/                 # frozen ns-3 module and built artifacts
├── campaigns/                      # per-campaign Bash wrappers
├── config/
│   └── experiment_matrix.yaml      # canonical experiment matrix
├── datasets/
│   ├── raw/                        # one CSV per simulator invocation
│   ├── merged/                     # one merged CSV per campaign
│   └── processed/                  # cleaned and aggregated datasets
├── figures/
│   ├── scripts/                    # IEEE-style matplotlib scripts
│   └── output/                     # generated PDF figures
├── tables/
│   └── paper_tables/               # CSV and LaTeX paper tables
├── scripts/                        # campaign runner, cleaners, aggregators, tables
├── validation/
│   ├── theory/                     # ALOHA and PER-vs-SNR consistency scripts
│   ├── experimental/               # place measured calibration datasets here
│   └── references/                 # reference CSVs used by validation
└── docs/
    └── paper_consistency_audit.md  # text revisions aligned with the frozen simulator
```

## System requirements

- Linux shell environment
- Bash 4+
- Python 3.10+
- `pandas`, `numpy`, `matplotlib`, `pyyaml`
- a compiled frozen simulator binary for `lpwan-pqc-export-exchanges.cc`

Install Python dependencies with:

```bash
python3 -m pip install pandas numpy matplotlib pyyaml
```

## Simulator hookup

Set the simulator binary path. The default expected location is:

```bash
export SIM_BIN=./build/scratch/ns3.47-lpwan-pqc-export-exchanges-default
```

If you prefer to call the scratch program through a wrapper, you may also set:

```bash
export SIM_RUNNER="./ns3 run"
export SIM_BIN="lpwan-pqc-export-exchanges"
```

Direct binary execution is recommended because it avoids quoting ambiguity.

## What the artifact reproduces

The experiment matrix covers five families:

1. baseline operation
2. bursty traffic phases
3. explicit collision modeling
4. RX duty-cycle effects
5. ACK batching effects

The main sweep dimensions are:

- spreading factor (`sf`)
- offered load proxy (`lambda_total`, exported again as `offered_load_eps` for plotting)
- burst ON/OFF parameters
- RX window and idle durations
- ACK batch size
- pacing strategy
- random seed / run

## Run everything

From the repository root:

```bash
./run_all_campaigns.sh
```

To control parallelism:

```bash
JOBS=8 ./run_all_campaigns.sh
```

This single command will:

1. generate a run matrix for each campaign,
2. execute all simulator runs,
3. save one raw CSV per exchange in `datasets/raw/`,
4. merge campaign CSVs into `datasets/merged/`,
5. clean and validate the schema,
6. build processed datasets in `datasets/processed/`,
7. regenerate all figures in `figures/output/`,
8. regenerate paper tables in `tables/paper_tables/`, and
9. run theoretical validation summaries in `validation/theory/`.

## Individual campaigns

```bash
./campaigns/campaign_baseline.sh
./campaigns/campaign_bursty.sh
./campaigns/campaign_collisions.sh
./campaigns/campaign_rxsleep.sh
./campaigns/campaign_ackbatch.sh
```

## Processed datasets generated

Representative processed outputs are:

- `datasets/processed/per_vs_load.csv`
- `datasets/processed/latency_vs_sf.csv`
- `datasets/processed/energy_per_exchange.csv`
- `datasets/processed/collision_probability_comparison.csv`
- `datasets/processed/rx_duty_cycle_impact.csv`
- `datasets/processed/ack_batching_impact.csv`
- `datasets/processed/summary_by_campaign.csv`

## Figures regenerated

The artifact produces IEEE-style PDF figures:

- `figures/output/per_vs_offered_load.pdf`
- `figures/output/exchange_latency_vs_sf.pdf`
- `figures/output/energy_per_exchange.pdf`
- `figures/output/collision_probability_comparison.pdf`
- `figures/output/rx_duty_cycle_impact.pdf`
- `figures/output/ack_batching_impact.pdf`

## Tables regenerated

The artifact exports both CSV and LaTeX tables:

- `tables/paper_tables/table_latency.csv`
- `tables/paper_tables/table_latency.tex`
- `tables/paper_tables/table_energy.csv`
- `tables/paper_tables/table_energy.tex`
- `tables/paper_tables/table_reliability.csv`
- `tables/paper_tables/table_reliability.tex`

## Validation and calibration

### Theoretical validation

- unslotted ALOHA collision approximation against simulator collision diagnostics
- PER-vs-SNR consistency check against reference curves

### Experimental calibration

Put published or measured gateway datasets into `validation/experimental/` and compare them with:

- `per_observed`
- `exchange_latency_s`
- `energy_j_exchange`
- `retransmissions_count`
- sleep-related and ACK-related counters

## Notes on the frozen simulator contract

- one simulator run must emit exactly one `ExchangeResult` row
- the stable CSV header is part of the contract
- receiver sleep is deterministic and separate from the stochastic failure rule
- the stochastic rule is `p_total = 1 - (1 - p_phy)(1 - p_collision)`
- the artifact adds metadata columns such as `campaign`, `run_id`, and `lambda_total` **after** the simulator run, without touching simulator code
