# 2026.07.12-beta.11

Status: early public beta, code complete and awaiting supervised powder testing.

## Purpose

Beta 11 is a focused response to the first 29-throw beta10 RL17 powder session. It removes the coarse behavior responsible for fast overthrows and replaces slow recovery retries with measured doses while keeping characterized motor speeds unchanged.

## Powder Evidence

The beta10 session contained 29 complete production throws at `37.50gn`:

- 16 good, 13 over, and no underthrows.
- Median `9.44s`; 17/29 below 10 seconds.
- Ten throws completed without recovery in a median `5.12s`, but eight overthrew.
- Seven of those ten had already landed over target after coarse/top-up delivery.
- Recovery produced 14/19 good results, but six stalled throws averaged `18.76s`.
- Powered coarse top-up appeared on all 29 throws and the latest coarse-late rate reached `91.7%`.
- The final eight throws were all accurate, but their median remained `10.46s` because seven depended on recovery.

The hardware is fast enough. The remaining problem is controlling coarse powder already in flight and delivering recovery powder efficiently.

## Firmware Changes

- Removed powered coarse top-up from production control.
- Added a fixed-duration motor-off `coarse_settle` observation after the normal settle detector, bounded by calibrated coarse settle timing.
- Preserved the highest valid weight during coarse tail drain so a temporary scale dip cannot hide arriving powder.
- Kept existing coarse telemetry as the initial bulk stop through complete passive settle.
- Removed the `coarse_topup` phase and added recovery pulse-count telemetry.
- Stopped normal production throws from mutating saved characterization and machine-calibration fields.
- Rebuilds derived model fields from saved characterization/calibration and resets obsolete production bias.
- Uses rolling production percentiles directly for fine-stop guards after sufficient observations exist.
- Computes effective recovery flow from delivered recovery weight divided by motor-on time.
- Exposes effective recovery-flow count, P25, and median through REST history.
- Replaced repeated continuous/micro recovery with approach, middle, and micro dose zones.
- Escalates dose after a verified no-progress pulse and stops after eight pulses or 15 seconds.

## Expected Behavior

- Main coarse and fine motor speeds remain unchanged.
- Coarse delivery should take roughly the same bulk time, then spend about `0.8-1.4s` observing residual powder instead of adding a powered top-up.
- More throws should enter fine control instead of finishing immediately from a late coarse landing.
- Recovery should use fewer, larger evidence-based doses instead of repeated ineffective micro pulses.
- The saved characterization should remain stable across production throws; only the rolling 24-observation runtime window changes.

## Verification Before Powder

- Pico 2 W incremental release build succeeds without new compiler warnings.
- Collector schema 4 passes all six tests, including the historical 77-throw replay baseline.
- DataLab measured-dose self-test passes all approach, middle, micro, and no-progress escalation bounds against 16 observed recovery-flow samples.
- Persistent AI history revision and binary layout remain unchanged.
- Original collector sessions and DataLab source evidence remain untouched.
- Full clean build, OTA, device identity, history preservation, and physical powder testing remain pending.

## Powder-Test Protocol

1. Keep the existing RL17 characterization, calibration, tube, and target unchanged.
2. Collect 12 supervised throws before considering any re-characterization.
3. Confirm no production row enters a `coarse_topup` phase.
4. Compare complete coarse settle, fine-entry reserve, recovery pulse count, recovery motor time, final error, and total time.
5. Stop immediately after a severe overthrow or any unresolved underthrow.
6. Require a material reduction in fast coarse overthrows without losing the sub-10-second path.

## Known Risks

- Historical data cannot predict how much of beta10's top-up-phase rise was residual bulk powder versus powered top-up delivery.
- Removing top-up may shift several grains of work to the fine motor until new evidence confirms the passive coarse landing.
- Stronger measured recovery doses require supervised validation before unattended use.
- The eight-pulse/15-second recovery bound can still end under target if mechanical flow stops completely.
