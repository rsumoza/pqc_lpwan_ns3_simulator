# Validation and Calibration Methodology

## Layer 1 — Theoretical validation

The artifact compares simulator outputs against two references:

1. **Analytical unslotted ALOHA collision approximation**
   \[
   p_{coll} pprox 1 - e^{-2G}, \qquad G = rac{\lambda_{total} T_{air}}{M}
   \]
   where `lambda_total` is the aggregate interferer attempt rate, `T_air` is mean fragment airtime, and `M` is the number of logical channels.

2. **PER-vs-SNR consistency check**
   The simulator's PHY-only error component (`per_channel_avg`) is checked against a reference table derived from LoRa PER/FER literature and from the soft-threshold behavior encoded in the simulator contract.

## Layer 2 — Experimental calibration

Calibration inputs should be stored in `validation/experimental/` and compared against processed datasets using the same CSV schema. The recommended approach is:

- align scenario labels (`environment`, `sf`, `payload_bytes_effective`)
- align SNR operating points or derived link budget
- compare `per_observed`, `exchange_latency_s`, `energy_j_exchange`, and retransmission statistics
- keep PHY calibration separate from collision and sleep calibration

## References to report in the paper

- Pure/unslotted ALOHA throughput and vulnerable-period collision analysis
- Guo et al. LoRa PER vs SNR experimental measurements
- Afisiadis et al. coded LoRa FER/PER analytical characterization
- LoRaWAN Regional Parameters for payload-size and duty-cycle constraints

The scripts here implement *consistency validation*, not simulator modification.
