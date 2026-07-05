# 2026.06.11-beta.1 Release Notes

Status: superseded by `2026.06.11-beta.2` before powder testing.

## Purpose

This beta addressed private testing where accuracy improved late in a run, but coarse handoff moved progressively earlier and micro recovery became slower. It preserved existing characterization and machine calibration data.

## Baseline Summary

- 77 production charges reviewed from private beta telemetry.
- 47 good, 27 over, and 3 under at +/-0.0205 gn.
- 10.71 second average and 10.36 second median.
- 33 charges at or below 10 seconds.
- 27 charges at or below 9 seconds.
- 14 of the final 17 charges were good, but those 17 averaged 11.97 seconds.

## Firmware Fixes

### Undercharge Healing

- Every settled undercharge now re-enters fine recovery.
- The direct fine-threshold exit can no longer abandon a charge just under target.
- Recovery retains the 30 second safety timeout and physical abort behavior.

### Phase-Specific Learning

- The first fine stop is retained as fast-finish telemetry.
- Later micro-recovery stops no longer overwrite fast-finish tail data.
- Recovery overthrows update micro tail and recovery speed only.
- Fast-fine overthrows update fast-fine tail and fine safety only.
- Coarse handoff learns independently from the settled post-coarse weight.
- Coarse runtime bias no longer changes fine-window or fine-stop margins.

### Persistent Runtime Learning

- Model sanitization no longer runs after every production throw.
- Learned recovery speed, flow, and tail are preserved across boot sanitization.
- Runtime recovery changes no longer snap back to characterization defaults.
- Tail estimates use asymmetric moving updates instead of permanent maximums.
- Inflated coarse-tail estimates converge toward observed production tails faster.

### Fine-Tube Compatibility

- Tube classification is now based on learned micro tail and micro flow.
- A large fast-fine tail alone no longer forces stop-and-settle high-tail recovery.
- Existing contaminated micro tail is bounded against characterization evidence.
- Safety bias relaxes toward the confidence-derived value instead of resetting.

### Release Identity

- Display and REST system information report `2026.06.11-beta.1`.
- Build remains Pico 2 W / RP2350 ARM-S, Release configuration.

## Verification

- RP2350 release build passed.
- Python compile checks passed for support tooling available at the time.
- Historical telemetry replay kept the corrected production result stable.
- Replay rejected 345 cup movement/touch samples.
- OTA staging, apply, and post-reboot version checks passed in private testing.
- Saved profile remained valid and enabled.

## Known Risks

- A later code review found that active recovery could repeatedly settle without feeding when slightly under target. Beta.2 fixed this edge case.
- This controller change required physical powder testing; replay validated scoring and telemetry interpretation, not future motor behavior.
- The private baseline contained one genuine fast-fine outlier. The new estimator reacts without permanently adopting the maximum, but this remained high priority to watch.
- Characterization coarse-tail snapshots remain physically inflated by scale lag. Production learning converges away from them, but characterization telemetry should eventually distinguish transport delay from powder flight.
- Steering controls remained beta and were not exercised in the baseline.

## Test Focus

- Confirm that no charge exits under target without recovery.
- Watch whether post-coarse settled weight converges toward target minus 2.5%.
- Compare recovery motor-on time against the previous private baseline.
- Confirm that balanced/low-tail recovery runs continuously rather than pulsing.
- Watch for any fast-fine jump larger than 0.20 gn after motor stop.
