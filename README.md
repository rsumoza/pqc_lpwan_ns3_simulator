# Temporal Dynamics of Fragmented PQC over LPWAN

## Overview

This repository provides a **fully reproducible experimental pipeline** for the paper:

> Temporal Alignment Shapes Finite-Horizon Feasibility of Fragmented Post-Quantum Exchanges over LoRa LPWANs

It enables any researcher to:

- reproduce all experiments,
- regenerate all datasets,
- rebuild all figures,
- recreate the results presented in the paper.

The artifact is built around a **frozen ns-3 simulator** and a **single unified pipeline script**, ensuring reproducibility and consistency.

---

## 🔁 One-Command Reproducibility

### 1. Build simulator (once)

```bash
./ns3 build lpwan-pqc-export-exchanges
```

### 2. Run full pipeline

```bash
bash scripts/bash/run_temporal_exploration_pipeline.sh \
  --mode full \
  --suite all \
  --jobs 1 \
  --no-build
```

This command will:

- run all simulation campaigns
- generate raw datasets
- merge CSV outputs
- compute processed summaries
- generate all IEEE-style figures

---

## 📁 Repository Structure

```
repo/
├── scripts/
│   ├── bash/
│   │   └── run_temporal_exploration_pipeline.sh
│   └── python/                  # auto-generated plotting scripts
│
├── simulator/
│   └── ns3-module/              # frozen ns-3 module and binaries
│
├── data/
│   ├── raw/                     # one CSV per simulation run
│   ├── merged/                  # merged datasets per campaign
│   └── processed/               # aggregated datasets
│
├── figures/                     # IEEE-style figures
│
├── logs/                        # execution logs
│
├── validation/
│   ├── theory/
│   ├── experimental/
│   └── references/
│
└── docs/
    ├── methodology.md
    └── validation.md
```

---

## ⚙️ System Requirements

- Linux
- Bash 4+
- Python 3.10+
- ns-3 installed

### Python dependencies

```bash
pip install pandas numpy matplotlib
```

---

## 🧪 Experimental Design

- Fragment payload: 20 bytes
- Duty-cycled receiver
- Offered load: λ ∈ {0, 0.5, 1, 2}
- SF ∈ {7, 9, 12}
- ECW ∈ {3, 5, 8}

---

## 📊 Campaigns

| Suite        | Description |
|--------------|------------|
| structural   | Lock-in |
| main         | Load sweep |
| robustness   | SF sweep |
| ecw          | ECW sensitivity |
| validation   | External validation |
| all          | All |

---

## ▶️ Execution Modes

### Full

```bash
--mode full
```

### Campaigns

```bash
--mode campaigns
```

### Figures

```bash
--mode figures
```

---

## 📊 Outputs

```
data/raw/
data/merged/
data/processed/
figures/
```

Includes:

- success_vs_lambda_three_pacing.pdf
- latency_per_success_three_pacing.pdf
- energy_per_success_three_pacing.pdf
- success_vs_sf_lambda_*.pdf
- success_vs_ecw.pdf

---

## ⚠️ Resource Usage

Recommended:

```bash
--jobs 1
```

---

## 📎 Validation

Place datasets in:

```
validation/experimental/
```

---

## 🔬 Reproducibility Contract

- one run → one CSV row
- stable schema
- no simulator modification

---

## 📜 Notes

- No PQC modification
- Focus on communication feasibility
- Temporal dynamics matter

---

## 👤 Author

Rodolfo Sumoza

---

## 📚 Citation

(To be added)
