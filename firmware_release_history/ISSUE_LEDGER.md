# Firmware Issue Ledger

Statuses:

- `open`: understood problem without a completed code fix.
- `awaiting-powder-test`: code and static checks pass, but physical behavior is unproven.
- `verified-static`: confirmed by build, lint, compile checks, or code audit.
- `verified-replay`: confirmed by replaying private historical telemetry through the new logic.
- `verified`: confirmed by a relevant physical powder session.
- `monitoring`: improved, but additional sessions are needed for confidence.

| ID | Status | Introduced or observed | Fixed in | Issue and evidence |
|---|---|---|---|---|
| OT-AI-001 | monitoring | Private steering-beta test | 2026.06.11-beta.2 | Fast-fine produced one genuine high outlier. Phase-specific EWMA now responds without permanently adopting the maximum. |
| OT-AI-002 | open | Characterization data | - | Coarse-tail snapshots combine transport delay, scale lag, and powder flight. Production convergence is improved, but characterization should eventually measure these separately. |
| OT-AI-003 | awaiting-powder-test | 2026.06.11-beta.1 audit | 2026.06.11-beta.2 | Active recovery could settle with its motor off slightly under target. The stop threshold now applies only after recovery reaches acceptance tolerance. |
| OT-AI-004 | awaiting-powder-test | Private steering-beta test | 2026.06.11-beta.1 | Micro-recovery stops overwrote fast-fine stop telemetry and contaminated the learned fast tail. The phases now retain separate telemetry. |
| OT-AI-005 | awaiting-powder-test | Private steering-beta test | 2026.06.11-beta.1 | Per-charge sanitization restored characterization defaults and erased runtime recovery adaptation. Sanitization now occurs only at model boundaries and preserves valid learned fields. |
| OT-AI-006 | awaiting-powder-test | Private steering-beta test | 2026.06.11-beta.1 | Coarse runtime bias leaked into fine-window and fine-stop calculations, slowing unrelated phases. Runtime corrections are now phase-specific. |
| OT-AI-007 | awaiting-powder-test | Private recovery/top-up test | 2026.06.16-beta.8 | Small underthrows could stop healing too early while coarse top-up could still be too assertive. Recovery is now split into `fine_recover` and `micro_heal`, and coarse top-up is reduced. |
| OT-LOG-001 | verified-replay | Private telemetry replay | 2026.06.11-beta.1 | Cup-lift/touch spikes became false final charge results. Stable post-finish weight is authoritative and cup movement artifacts are rejected in replay. |
| OT-LOG-002 | verified-static | Private beta analysis | 2026.06.11-beta.1 | Internal analysis output lacked speed bands, recovery timing, stalls, artifact counts, and learned fast/micro model fields. These are now available to private validation tooling. |

When a physical test verifies or disproves an entry, update its status, summarize the evidence without local paths, and reference the issue ID in the next release notes.
