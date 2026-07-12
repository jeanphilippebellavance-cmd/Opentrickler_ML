# Firmware Changelog

## 2026.07.12-beta.11 - Passive Coarse Settle And Measured Recovery

- Removed powered coarse top-up from production throws after beta10 used it on every RL17 throw and frequently landed over target before fine control.
- Added a complete `coarse_settle` motor-off observation using calibrated coarse settle timing.
- Stopped production observations from rewriting characterization, calibration, tail, handoff, and runtime-bias fields.
- Made recent production percentiles the authority for temporary fine-stop guards once sufficient evidence exists.
- Added effective recovery-flow P25 and median statistics from the rolling observation window.
- Replaced continuous/repeated micro recovery with bounded three-zone measured doses.
- Added no-progress dose escalation with an eight-pulse and 15-second recovery bound.
- Added recovery pulse-count telemetry for direct validation of the new retry bound.

## 2026.07.12-beta.10 - Automatic Finish Safety

- Removed manual Faster, Safer, Fine Finish Faster, Bulk Closer, and Undo steering controls.
- Removed the `/rest/ai_steering` API and the corresponding model fields while preserving the persistent-history layout.
- Added phase-specific recent-history statistics for fast finishes, recovery finishes, coarse late landings, and complete fast-finish tail.
- Made unstable or high-overthrow windows widen finish protection instead of disabling percentile guards.
- Added a bounded motor-off tail-drain observation before recovery starts near target.
- Reduced recovery pulse authority when recent recovery finishes show elevated overthrow risk.
- Kept characterized motor speeds unchanged; beta 10 spends additional time only at risky handoffs and finish transitions.

## 2026.07.10-beta.9 - Production-Tail Controller

- Replaced the fixed coarse-tail rejection rule with a target-aware time-versus-risk selector for new characterizations.
- Made characterization fail closed when no coarse candidate satisfies the new tail limits.
- Preserved saved motor choices across firmware boots; the new selector takes effect only after a fresh characterization.
- Separated production-speed and low-speed machine-calibration statistics.
- Removed duplicate scale-latency allowances from stop-to-settle tail planning.
- Added robust per-profile runtime statistics over the latest 24 observations.
- Allowed stable production tails to tighten coarse and fine handoffs while retaining percentile-based safety floors.
- Made negative runtime bias effective instead of silently discarding it.
- Added direct learning from the pre-recovery fine landing and retained recovery as the underthrow backstop.
- Exposed runtime tail, landing, timing, recovery, overthrow, and underthrow statistics through REST history.
- Cleared only the affected profile's runtime window after applying a new characterization or calibration.

## 2026.06.16-beta.8 - Recovery Split And Safer Top-Up

- Split final recovery into `fine_recover` and `micro_heal` zones.
- Reduced coarse top-up pulse count, duration, and desired drop size.
- Preserved first fine-stop tail telemetry instead of overwriting it with recovery delivery.
- Widened final-weight plausibility handling so severe overcharges remain visible to runtime learning.
- Adjusted recovery learning so underthrows can increase recovery authority even when motor-on time was low.
- Stopped active recovery delivery from being treated as passive fast-fine tail.

## 2026.06.11-beta.2 - Recovery Dead-Zone Fix

- Includes all phase-isolated learning changes from beta.1.
- Allows active micro recovery to continue feeding when the remaining deficit is between acceptance tolerance and the legacy fine-stop threshold.
- Superseded beta.1 before powder testing.

## 2026.06.11-beta.1 - Phase-Isolated Learning

- Made settled undercharges re-enter fine recovery instead of exiting early.
- Preserved the first fast-fine stop separately from micro-recovery telemetry.
- Stopped coarse runtime bias from changing fine stopping decisions.
- Stopped recovery overthrows from inflating the fast-fine tail model.
- Replaced maximum-only tail learning with asymmetric moving estimates.
- Preserved learned recovery speed, flow, and tail across model sanitization.
- Based fine-tube compatibility on micro tail rather than fast-fine tail.
- Accelerated convergence from inflated characterization coarse-tail estimates.
- Added explicit release identity to display and REST system information.
- Corrected telemetry scoring for cup-lift artifacts and settled undercharges.
- Added production median, speed bands, mean error, recovery use, and stall metrics to beta analysis output.
- Superseded by beta.2 after a code audit found a slight-under-target recovery dead zone.

## 2026.06.01 Steering Beta - Reconstructed

- Added user steering actions: Faster, Safer, Fine Finish Faster, Bulk Closer, and Undo Last Steer.
- Added learned fast-finish and micro-finish flow/tail fields.
- Added tail-confidence and fine-tube compatibility reporting.
- Added model-based fine stop margins and adaptive micro recovery.
- Added OTA firmware update support.

Exact source identity is unavailable because this version was not committed.

## 2026.05.31 Stable Before Steering Beta - Reconstructed

- Preserved as the rollback point immediately before steering and learned-tail work.
- Includes coarse trim/glide and fine recovery behavior developed during May testing.

Exact source identity is unavailable because this version was not committed.

## Earlier AI Development - Validation Evidence Only

April and May 2026 beta testing covered the progression from pulsed fine finishing through undercharge healing, machine calibration, OTA safety fixes, runtime guards, tail-safe control, and coarse trim glide. Raw test data is intentionally not committed to this repository.
