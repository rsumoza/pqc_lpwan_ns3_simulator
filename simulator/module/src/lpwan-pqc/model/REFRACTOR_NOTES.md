# Refactor notes

This delivery is intentionally conservative.

It keeps the simulator architecture intact while applying safe structural fixes:
- corrected pacing default activation
- renamed RX-window helper to match full-containment semantics
- removed non-contractual verbose compute logs
- preserved CSV schema and ExchangeResult layout
- preserved CLI reproducibility behavior

It does **not** attempt a risky semantic rewrite of the exchange protocol core.
That choice was made to maximize compileability and behavioral continuity for ns-3 campaigns.
