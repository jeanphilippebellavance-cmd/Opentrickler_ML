# 2026.06.16-beta.8

Status: early beta, private validation in progress.

## Purpose

Beta 8 improves final recovery behavior after private beta testing showed that some charges could stop meaningfully under target while other fast throws finished slightly high.

## Data Reviewed

- Private beta RL17 recovery and coarse-top-up sessions.
- Live REST status after beta OTA validation.
- Code audit of final recovery, fine tail telemetry, and runtime learning.

## Firmware Changes

- Split final recovery into a larger `fine_recover` approach zone and a true `micro_heal` zone.
- Fixed an underthrow stall where recovery could time out with almost no fine motor runtime.
- Changed micro-heal feed logic so it feeds only to a learned stop margin, then settles and re-checks instead of driving all the way to the exact target.
- Reduced coarse top-up pulse count, duration, and desired drop size to lower the risk of jumping past target.
- Preserved fine tail telemetry as the first fine-stop settle tail instead of overwriting it with final or post-finish weight.
- Widened production final-weight plausibility limits so severe overcharges remain visible to AI learning.
- Adjusted runtime learning so recovery-related underthrows increase recovery authority even when the failure had very little motor-on time.
- Stopped active recovery delivery from being counted as fast fine tail in AI learning.

## Expected Behavior

- A charge stuck roughly `0.10-0.18gn` under target should continue healing instead of asking for cup removal.
- Fine recovery should approach in two stages: safely close the gap, settle, then micro-heal.
- Coarse top-up should still improve speed but be less likely to create a lost charge before fine control starts.
- Future learning should better distinguish true passive tail from powder intentionally added during recovery.

## Test Status

- Built successfully for Pico 2 W release.
- Private OTA validation completed successfully.
- Device-reported version matched `2026.06.16-beta.8` after reboot.
- Additional powder verification remains required before wider release.

## Known Risks

- This remains early beta firmware and needs more powder validation.
- Coarse-tail snapshots still combine scale lag, transport delay, and powder flight.
- Steering controls should be applied one step at a time and observed carefully.
- Saved models are profile-specific and should not be copied blindly between different powder or tube setups.
