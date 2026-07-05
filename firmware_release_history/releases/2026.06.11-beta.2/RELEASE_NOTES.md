# 2026.06.11-beta.2 Release Notes

Status: installed private beta, pending broader powder validation.

## Purpose

This release corrects learning contamination and recovery behavior observed during private steering-beta testing. It keeps the strong late-session accuracy while targeting shorter recovery and eliminating settled undercharge exits.

Beta.2 superseded beta.1 before powder testing. A final control-flow audit found that beta.1 could repeatedly settle with the motor off when active recovery was slightly under target.

## Baseline Summary

- 77 production charges reviewed from private beta telemetry.
- 47 good, 27 over, and 3 under at +/-0.0205 gn.
- 10.71 second average and 10.36 second median.
- 33 charges at or below 10 seconds.
- 27 charges at or below 9 seconds.
- Mean final error was +0.0173 gn.
- Recovery was used by 73 charges and added 2.8 seconds on average.
- The final 17 charges were 14 good and 3 over, averaging 11.97 seconds.

## Firmware Changes

### Undercharge Healing

- Every settled undercharge now re-enters fine recovery.
- Active recovery continues feeding until it reaches acceptance tolerance.
- The legacy fine-stop threshold can no longer create a motor-off dead zone.
- The direct fine-threshold exit now settles, records its reason, and resumes recovery when the stable weight remains under target.
- Recovery retains its 30 second safety timeout and physical abort behavior.

### Phase-Isolated Learning

- The first fine stop is retained as fast-finish telemetry.
- Micro recovery has separate start, end, post-peak, motor-time, stall, and exit telemetry.
- Recovery overthrows update micro tail and recovery speed only.
- Fast-fine overthrows update fast-fine tail and fine safety only.
- Coarse handoff learns independently from settled post-coarse weight.
- Positive coarse runtime bias no longer changes fine-window or fine-stop margins.
- Final overthrows no longer globally slow unrelated phases.

### Adaptive Model Behavior

- Per-charge model sanitization was removed so learned recovery speed, flow, and tail no longer snap back to characterization defaults.
- Boot sanitization preserves valid runtime recovery learning.
- Fast, micro, recovery, coarse, and trim tails use bounded moving estimates rather than permanent maximums.
- Inflated characterization coarse tails converge faster toward production data.
- Coarse runtime bias is limited to a phase-appropriate maximum.
- Fine safety bias converges smoothly and decays faster when inherited data is overly conservative.
- Fine-tube classification uses learned micro tail and micro flow.

## Compatibility

- AI history revision remains 14.
- Saved characterization, machine calibration, and steering state are retained.
- No mandatory characterization or calibration is expected after OTA.
- Existing runtime bias is clamped on boot; old tail estimates then converge from correctly separated production telemetry.

## Verification

- Pico 2 W / RP2350 ARM-S release build passed.
- C/C++ diff whitespace validation passed.
- Python compile validation passed for support tooling available at the time.
- Historical telemetry replay kept the corrected production result stable.
- Replay rejected 345 cup/touch artifacts.
- Binary release identity reported `2026.06.11-beta.2`.
- OTA and post-reboot checks passed in private testing.

## Known Risks

- Physical powder testing is still required; replay validates scoring and telemetry interpretation, not future motor behavior.
- The baseline includes one genuine fast-fine outlier. The estimator now reacts without permanently adopting that maximum, but this remains important to watch.
- Characterization coarse-tail snapshots still combine scale lag, transport delay, and powder flight. Production learning now converges away from inflated snapshots, but characterization telemetry should eventually separate them.
- Steering controls remain beta.

## Test Focus

- Confirm that no stable undercharge asks for cup removal before recovery.
- Watch slight under-target recovery for continuous progress.
- Compare recovery motor-on time against the previous private baseline.
- Confirm coarse handoff moves toward the target-minus-2.5-percent region.
- Watch for delayed fast-fine tail larger than 0.20 gn.
- Confirm good/red display status follows the final stable post-finish weight.
