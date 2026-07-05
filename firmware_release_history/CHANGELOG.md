# Firmware Changelog

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
- Added production median, speed bands, mean error, recovery use, and stall metrics to private analysis output.
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

## Earlier AI Development - Private Evidence Only

April and May 2026 private testing covered the progression from pulsed fine finishing through undercharge healing, machine calibration, OTA safety fixes, runtime guards, tail-safe control, and coarse trim glide. Raw test data is intentionally not committed to this repository.
