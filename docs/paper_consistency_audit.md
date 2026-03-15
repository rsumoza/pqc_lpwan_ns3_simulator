# Paper Consistency Audit Against the Frozen Simulator

## Scope

This audit was derived from the supplied simulator contract, the frozen module sources, and the scratch driver `lpwan-pqc-export-exchanges.cc`. The simulator is treated as immutable.

## 1. Confirmed simulator semantics

### Fragmentation model

The simulator executes one full exchange per run, with:

- `n_fragments_exchange = pk_fragments + ct_fragments`
- one CSV row per completed exchange
- separate export of PK and CT fragment counts

This is consistent with a fragment-based PQC transport model and the paper should describe the exchange exactly in those terms.

### Failure model

The stochastic failure rule implemented by the simulator is:

\[
 p_{total} = 1 - (1 - p_{phy})(1 - p_{collision})
\]

where:

- `p_phy` corresponds to the PHY/channel decoding component (`per_channel_avg` in the export)
- `p_collision` corresponds to the collision component (`per_collision_sim_avg` or `per_collision_th_avg` depending on the diagnostic used)

This rule is implemented before any deterministic receiver-availability gating.

### RX duty-cycle model

The reception contract is stricter than a generic overlap rule. A frame is received only if **its full airtime interval fits entirely inside an active RX window**. Partial overlap is not sufficient. This is exactly consistent with the contract text you supplied and with `DoesIntervalOverlapRxWindow(...)` in the code.

### ACK batching model

ACK behavior is batch-oriented, not stop-and-wait. The export includes:

- `ack_batch_size`
- `ack_frames_tx`
- `ack_logical_failures`
- `ack_batch_retx_rounds`
- `lost_ack_sleep`
- `lost_ack_channel`

The paper should therefore refer to **ACK resolution rounds** and **batched ACK exposure**, not per-fragment ACK semantics.

### Pacing strategies

The scratch driver and module expose the pacing modes:

- `baseline`
- `fixed`
- `radioaware`
- `radioaware_phase_bounded`

The contract language requested “baseline / radio-aware / phase-aware”. For consistency with the frozen code, the paper should name the implemented policies using the exact code-level identifiers, while explaining that `radioaware_phase_bounded` is the phase-aware bounded pacing policy.

### Collision model

The frozen simulator supports both:

- analytical collision probability mode
- explicit event-driven collision mode

through `GetCollisionProbabilityAt(...)` and the `useExplicitEventCollisions` switch. The paper should explicitly state that explicit collisions are a simulator mode, not a post-processing approximation.

## 2. Required paper updates

### Update A — state the failure equation explicitly

Recommended text:

> For transmission attempts issued while the receiver is available, the simulator combines PHY/channel and collision failures as \(p_{total} = 1 - (1-p_{phy})(1-p_{collision})\). This term represents the stochastic failure probability of an attempted frame transmission.

### Update B — separate receiver sleep from stochastic failure

Recommended text:

> Receiver sleep is not folded into \(p_{total}\). Instead, receiver duty cycling is modeled as a deterministic availability constraint: a frame is considered decodable only if its complete airtime interval lies fully within an active RX window.

### Update C — normalize pacing terminology

Recommended text:

> The implemented pacing policies are `baseline`, `radioaware`, and `radioaware_phase_bounded`. In descriptive text, the last of these may be referred to as a bounded phase-aware radio pacing strategy, but figures and tables should preserve the exact exported `pacing_mode` names for reproducibility.

### Update D — correct metric naming in tables and figures

Use the exact CSV names when defining metrics in the methodology or appendices:

- `retransmissions_count` instead of `retransmissions`
- `exchange_latency_s` instead of `exchange_latency_ms`
- `per_channel_avg` for the PHY-only error component
- `per_collision_sim_avg` / `per_collision_th_avg` for collision diagnostics
- `per_total_avg` for the combined stochastic total

### Update E — define observed PER carefully

Recommended text:

> `per_observed` is an exchange-level observed fragment error ratio derived from the simulator export. It should not be conflated with the analytical PHY-only error component or with the total stochastic attempt-failure probability.

## 3. Text blocks ready to paste

### Channel and failure model paragraph

> The simulator separates three mechanisms affecting fragment delivery. First, PHY decoding failures are represented by a PHY-only error component, exported as `per_channel_avg`. Second, overlap-induced ALOHA losses are represented by collision terms exported as `per_collision_sim_avg` and `per_collision_th_avg`. For attempts issued while the receiver is available, these two stochastic components are combined as \(p_{total} = 1 - (1-p_{phy})(1-p_{collision})\). Third, receiver duty cycling is modeled independently as a deterministic availability gate. A data or ACK frame is considered receivable only if its complete airtime interval lies fully inside an active RX window; partial overlap does not count as successful reception.

### ACK batching paragraph

> Reliability is implemented through fragment-level retransmission together with batched acknowledgment rounds. After groups of up to `ack_batch_size` accepted data fragments, the receiver issues an ACK resolution round. This reduces reverse-link control traffic, but it also increases the amount of previously accepted state exposed to a single ACK loss or sleep-related miss.

### Reproducibility paragraph

> Each simulator run produces exactly one `ExchangeResult` CSV record, and the CSV schema is treated as part of the simulator contract. All processed datasets, figures, and paper tables in the artifact are generated from these one-row-per-exchange exports without modifying the simulator implementation.
