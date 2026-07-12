# Firmware Issue Ledger

Statuses:

- `open`: understood problem without a completed code fix.
- `awaiting-powder-test`: code and static checks pass, but physical behavior is unproven.
- `verified-static`: confirmed by build, lint, compile checks, or code audit.
- `verified-replay`: confirmed by replaying beta validation telemetry through the new logic.
- `verified`: confirmed by a relevant physical powder session.
- `monitoring`: improved, but additional sessions are needed for confidence.

| ID | Status | Introduced or observed | Fixed in | Issue and evidence |
|---|---|---|---|---|
| OT-AI-001 | monitoring | Steering-beta test | 2026.06.11-beta.2 | Fast-fine produced one genuine high outlier. Phase-specific EWMA now responds without permanently adopting the maximum. |
| OT-AI-002 | open | Characterization data | - | Coarse-tail snapshots combine transport delay, scale lag, and powder flight. Production convergence is improved, but characterization should eventually measure these separately. |
| OT-AI-003 | awaiting-powder-test | 2026.06.11-beta.1 audit | 2026.06.11-beta.2 | Active recovery could settle with its motor off slightly under target. The stop threshold now applies only after recovery reaches acceptance tolerance. |
| OT-AI-004 | awaiting-powder-test | Steering-beta test | 2026.06.11-beta.1 | Micro-recovery stops overwrote fast-fine stop telemetry and contaminated the learned fast tail. The phases now retain separate telemetry. |
| OT-AI-005 | awaiting-powder-test | Steering-beta test | 2026.06.11-beta.1 | Per-charge sanitization restored characterization defaults and erased runtime recovery adaptation. Sanitization now occurs only at model boundaries and preserves valid learned fields. |
| OT-AI-006 | awaiting-powder-test | Steering-beta test | 2026.06.11-beta.1 | Coarse runtime bias leaked into fine-window and fine-stop calculations, slowing unrelated phases. Runtime corrections are now phase-specific. |
| OT-AI-007 | awaiting-powder-test | Recovery/top-up test | 2026.06.16-beta.8 | Small underthrows could stop healing too early while coarse top-up could still be too assertive. Recovery is now split into `fine_recover` and `micro_heal`, and coarse top-up is reduced. |
| OT-AI-008 | awaiting-powder-test | Cross-profile collector audit | 2026.07.10-beta.9 | Bulk handoff counted stop-to-settle latency more than once, production learning could not lower the saved machine margin, and negative runtime bias was ignored by the planner. Beta 9 uses stable production-tail percentiles with explicit safety floors. |
| OT-AI-009 | awaiting-powder-test | Machine-calibration audit | 2026.07.10-beta.9 | Production-speed and low-speed calibration samples shared tail and uncertainty accumulators. Beta 9 isolates the two speed regimes and keeps native trim-tail evidence separate from high-speed bulk tail. |
| OT-AI-010 | verified-replay | Characterization and production replay | 2026.07.10-beta.9 | Coarse sample selection rejected every tail above `4gn` and ignored the configured time cost. The target-aware selector now minimizes expected bulk time plus tail risk; the DataLab replay confirms the selected candidates obey the new tail and delivered-mass limits. |
| OT-AI-011 | awaiting-powder-test | RL17 REST-history audit | 2026.07.12-beta.10 | Manual steering had reached aggressive limits while recent results were already fast and overthrow-prone. Beta 10 removes steering and lets profile evidence govern finish protection automatically. |
| OT-AI-012 | verified-replay | Phase-specific guard replay | 2026.07.12-beta.10 | An unstable fine-tail window disabled the percentile guard exactly when its upper tail was dangerous. Beta 10 permits risky evidence to widen protection even when the window is not stable enough to tighten it. |
| OT-AI-013 | awaiting-powder-test | Recovery-path audit | 2026.07.12-beta.10 | Recovery could start before residual powder from the fast finish had drained, causing avoidable overthrows and contaminating recovery delivery telemetry. Beta 10 observes a bounded motor-off drain before near-target recovery and reduces recovery pulse authority when that phase is risky. |
| OT-AI-014 | awaiting-powder-test | 29-throw beta10 RL17 session | 2026.07.12-beta.11 | Powered coarse top-up ran on all 29 throws; seven of ten no-recovery throws were already over target after the coarse landing. Beta 11 removes powered top-up and observes the complete passive coarse tail before fine control. |
| OT-AI-015 | awaiting-powder-test | 29-throw beta10 RL17 session | 2026.07.12-beta.11 | Six stalled recovery throws were accurate but averaged 18.76 seconds. Beta 11 calculates three-zone recovery doses from modeled and observed effective flow, escalates verified no-progress doses, and bounds recovery to eight pulses or 15 seconds. |
| OT-AI-016 | verified-static | Beta10 model-drift audit | 2026.07.12-beta.11 | Production learning moved persistent fine-tail estimates down while observed P95 increased and inflated machine handoff fields without correcting coarse late landings. Beta 11 keeps characterization stable and places production adaptation exclusively in the rolling runtime window. |
| OT-LOG-003 | verified-static | Runtime-statistics implementation | 2026.07.10-beta.9 | REST history now reports the exact 24-observation profile statistics used to unlock or withhold production-tail optimization. |
| OT-LOG-004 | verified-replay | Beta 10 runtime replay | 2026.07.12-beta.10 | REST history now separates fast-finish, recovery-phase, and coarse-late risk and reports complete fast-finish P90/P95 tail estimates used by the automatic guard. |
| OT-LOG-005 | verified-static | Beta11 telemetry contract | 2026.07.12-beta.11 | Charge REST exposes recovery pulse count and the new `coarse_settle` phase; runtime history exposes effective recovery-flow count, P25, and median. |
| OT-LOG-001 | verified-replay | Telemetry replay | 2026.06.11-beta.1 | Cup-lift/touch spikes became false final charge results. Stable post-finish weight is authoritative and cup movement artifacts are rejected in replay. |
| OT-LOG-002 | verified-static | Beta analysis | 2026.06.11-beta.1 | Internal analysis output lacked speed bands, recovery timing, stalls, artifact counts, and learned fast/micro model fields. These are now available to validation tooling. |

When a physical test verifies or disproves an entry, update its status, summarize the evidence without local paths, and reference the issue ID in the next release notes.
