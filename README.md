# OpenTrickler ML Firmware

Public beta firmware for the OpenTrickler Raspberry Pi Pico 2 W / RP2350 controller, with profile-aware flow characterization, machine calibration, automatic runtime learning, REST status endpoints, and OTA staging.

> Early beta: this firmware controls reloading equipment. Verify every charge with a calibrated scale, supervise all operation, and keep a known-good UF2 rollback image available.

## Project Status

- Current beta identity: `2026.07.12-beta.11`
- Supported controller: Raspberry Pi Pico 2 W / RP2350 only
- Unsupported controller: Raspberry Pi Pico W / RP2040
- Repository status: public beta, early release
- Documentation site: [docs/index.html](docs/index.html)

## Pico 2 W / RP2350 Only

This firmware intentionally does not support the original Pico W / RP2040 controller.

The current ML firmware combines AI characterization, runtime learning, REST telemetry, OTA staging, Wi-Fi portal code, scale drivers, and recovery logic. That feature set now exceeds the RP2040's 256 KB RAM budget; the Pico W build overflows RAM during linking. Pico 2 W / RP2350 provides 512 KB RAM and is the supported hardware baseline for this beta.

The build system enforces this. Any attempt to configure with `-DPICO_BOARD=pico_w` fails immediately with a clear message instead of producing a risky or partial firmware image.

## Recommended Fine Tube

This firmware pairs especially well with the ultra-low-flow small fine tube from the original Eamars OpenTrickler hardware repository: [Ultra Low Flow Fine Trickler Tube - Scaled to 99.7 Percent.stl](https://github.com/eamars/OpenTrickler/blob/main/STL/OpenTrickler/Ultra%20Low%20Flow%20Fine%20Trickler%20Tube%20-%20Scaled%20to%2099.7%20Percent.stl).

For this ML firmware, the ultra-low-flow tube is the preferred fine tube because its slower, smoother feed gives the controller far more precision during the final approach, recovery, and `micro_heal` phases. Other fine tube designs may work, but tube geometry changes flow and tail behavior. Create a separate profile and rerun characterization and machine calibration after changing tube design, powder, scale, or target range.

## What Is Different From Original OpenTrickler Firmware

This branch is based on the OpenTrickler controller firmware, but the ML beta target is Pico 2 W / RP2350 only. It adds an adaptive tuning layer around the existing charge workflow.

- Adds AI flow characterization for coarse, trim, fine, and micro-recovery behavior.
- Adds machine calibration for scale response, settle timing, open-loop flow, and tail behavior.
- Saves learned models per profile instead of relying only on fixed motor settings.
- Records runtime observations during normal throws and applies them as temporary phase-specific guards without rewriting characterization.
- Adds automatic phase-specific safety guards from recent coarse, fast-finish, and recovery results.
- Adds a motor-off `tail_drain` observation before recovery so powder already in flight is not mistaken for recovery delivery.
- Adds guarded pulse recovery when recent recovery behavior is overthrow-prone.
- Adds REST endpoints for AI status, model history, charge state, system information, and OTA staging.
- Adds a web-portal AI tuning panel for characterization, calibration, and save/apply.
- Moves the ML beta hardware baseline to Pico 2 W / RP2350 because RP2040 RAM is no longer sufficient.
- Fixes and hardens several AI telemetry and recovery behaviors found during beta testing.

## Feature Overview

### Profile-Aware AI Model

Each profile can hold its own learned model. Use separate profiles for different powders, tube setups, scale behavior, or hardware changes. A model that works well for one powder/setup should not be assumed to be correct for another.

### Characterization

Characterization measures how powder moves through the current machine and profile. It samples coarse and fine behavior, estimates flow and tail, builds a finish plan, and ends in a `ready_to_save` state when the working model can be applied to the selected profile.

In practical terms, characterization is the first learning pass for a powder/profile. It temporarily takes over charge mode and runs controlled motor pulses while the scale records how much powder actually arrived, how much arrived after the motor stopped, and how long the scale took to show the change.

Default characterization plan:

| Stage | Motor | Default samples | Default budget | Default sample target | What it learns |
| --- | --- | ---: | ---: | ---: | --- |
| `characterizing_coarse` | Coarse only | 12 | 120 gn | 8.0 gn | Bulk speed, coarse flow curve, coarse tail, and a lower-tail trim/coarse speed. |
| `characterizing_fine` | Fine only | 12 | 36 gn | 1.75 gn | Fine flow curve, fine tail, fast-finish behavior, and the first micro/recovery samples. |

The fine stage reserves its first four low-speed samples for recovery behavior. Those samples use fine speeds around `0.08`, `0.12`, `0.20`, and `0.35` rps with longer motor-on windows so the firmware can learn near-target `micro_heal` behavior instead of treating the fine tube as only a fast finishing motor.

Characterization produces a working model, not an automatically trusted model. Save it only after the session reaches `ready_to_save` and the samples were clean.

### Machine Calibration

Machine calibration runs after a characterization model exists. It measures scale/sample timing, response delay, settle behavior, coarse tail, fine tail, open-loop flow, and handoff margins so production throws can make safer decisions.

Machine calibration is the second pass. It starts from a saved characterization model, then measures the real machine timing around that model: scale lag, first response time, settle time, post-stop tail, uncertainty, open-loop coarse/fine flow, and safe handoff margins.

Default calibration plan:

| Stage | Motor | Default samples | Default budget guard | Motor-on pattern | What it learns |
| --- | --- | ---: | ---: | --- | --- |
| `calibrating_coarse` | Coarse only | 8 | max(80 gn, 65% of coarse budget) | 250, 450, 650, 850 ms at production speed, then repeated at trim speed | Scale response, coarse settle, coarse tail, trim/bulk flow, and uncertainty. |
| `calibrating_fine` | Fine only | 8 | max(12 gn, 45% of fine budget) | 500, 800, 1100, 1400 ms at fine speed, then 2500, 3500, 5000, 7000 ms at recovery speed | Fine response, fine settle, fine tail, micro-flow, and post-finish watch timing. |

Calibration needs at least three valid coarse and three valid fine samples. When it finishes, save/apply it just like characterization. The saved model then contains both powder-flow characterization and machine timing calibration.

### Runtime Learning

During normal production, the firmware observes final error, phase timing, stop weights, recovery delivery, and stall count. These observations drive a rolling runtime controller. Characterization and machine calibration remain stable until the operator explicitly reruns them.

### Automatic Runtime Safety

Beta 11 keeps runtime evidence separate from the characterized machine model:

- Stable, accurate history may tighten a stop guard.
- Unstable or overthrow-prone history may widen a guard immediately, but never tighten it.
- Fast-finish, recovery, and coarse-handoff outcomes are evaluated separately.
- Complete powder arrival after fine stop is included in the fast-finish tail guard.
- Powered coarse top-up is removed; the controller observes the complete motor-off coarse tail before fine control begins.
- Near-target underthrows wait through a calibrated motor-off tail-drain period before recovery starts.
- Recovery uses measured delivered flow to calculate bounded approach and micro doses.
- No-progress pulses increase dose within an eight-pulse and 15-second recovery bound.

Characterized production motor speeds remain unchanged while these guards are active. Runtime throws update only the rolling observation window, preventing production noise from drifting the saved characterization.

### Full AI Tuning Reference

The complete characterization, calibration, model, runtime observation, and REST parameter reference is published here:

- [AI tuning reference](https://whokilledbambilabs.github.io/Opentrickler_ML/ai-tuning-reference.html)

Key adjustable AI configuration parameters:

| Parameter | Default | Range | Meaning |
| --- | ---: | ---: | --- |
| `coarse_budget_gn` | 120 | 20-500 | Maximum total coarse powder used during characterization before the stage can stop. |
| `fine_budget_gn` | 36 | 5-150 | Maximum total fine powder used during characterization before the stage can stop. |
| `coarse_sample_count` | 12 | 2-12 | Planned coarse characterization samples. |
| `fine_sample_count` | 12 | 2-12 | Planned fine characterization samples. |
| `coarse_sample_target_gn` | 8.0 | 2-50 | Target delivered powder per coarse characterization sample before safety limits adjust it. |
| `fine_sample_target_gn` | 1.75 | 0.2-10 | Target delivered powder per fine characterization sample before safety limits adjust it. |
| `noise_margin` | 0.05 | 0.005-0.25 | Minimum useful scale movement used to filter tiny/noisy samples. |
| `time_cost_weight` | 1.0 | 0.1-20 | Model scoring weight for speed/time preference. |
| `error_cost_weight` | 8.0 | 0.1-50 | Model scoring weight for tail/error avoidance. |

Normal users should leave these defaults alone until they have a saved rollback model and enough repeatable test data to justify changing them.

### OTA Staging

The firmware can stage an `app.bin` over Wi-Fi, validate CRC32, and apply it on supported flash layouts. USB UF2 flashing remains the safest first-install and rollback method.

## Creating A Profile

1. Open the OpenTrickler web portal or the local controller UI.
2. Go to Settings and choose a profile slot.
3. Name the profile for the powder/setup, for example `Varget 36.5` or `H4350 short tube`.
4. Set or copy the baseline motor parameters you trust.
5. Save the profile before running characterization.
6. Keep one profile per powder/setup/tube/scale combination whenever practical.

When changing powder, tube geometry, scale, gate behavior, or target range, create a new profile or re-characterize the existing one.

## Running Characterization

1. Connect the controller to 2.4 GHz Wi-Fi and open the web portal.
2. Open Settings, then AI Tuning.
3. Select the correct profile.
4. Enter the normal target charge weight for that setup.
5. Confirm the scale is connected, stable, and zeroed with the pan workflow you normally use.
6. Start characterization and follow the normal remove/return pan prompts.
7. Let the controller complete the planned coarse and fine samples.
8. Wait for the state to reach `ready_to_save`.
9. Apply/save the model to the selected profile.

Do not use characterization results if the scale was unstable, powder bridged, the pan workflow was interrupted, or the wrong profile was selected.

## Running Machine Calibration

1. Save a characterization model first.
2. Select the same profile and target charge weight.
3. Start machine calibration from the AI Tuning panel.
4. Let the controller run the timing samples without changing the setup.
5. Save/apply the calibration when complete.
6. Run a small supervised production set and confirm behavior before trusting the model.

Calibration is most useful after hardware changes, tube changes, scale changes, or when handoff/tail behavior looks wrong.

## Basic Use

1. Select the correct profile.
2. Enter the target charge weight in grains.
3. Place the pan on the scale and wait for zero.
4. Let the controller charge, settle, and recover if needed.
5. Remove the pan only when prompted.
6. Verify every charge.
7. Watch for repeatable under/over behavior; re-characterize if the setup no longer matches the saved model.

## Build From Source

Install Git, CMake, Ninja, the ARM GCC toolchain, Python 3, and the Raspberry Pi Pico SDK. The included PowerShell helper expects the Pico SDK environment used by the Raspberry Pi Pico extension.

```powershell
git clone <repository-url-from-the-github-code-button>
cd Opentrickler_ML
git submodule update --init --recursive
.\configure_env.ps1
cmake -B build-pico2w-release -G Ninja -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico2_w
cmake --build build-pico2w-release --config Release
```

Useful outputs:

- `build-pico2w-release/app.uf2`: USB BOOTSEL flashing
- `build-pico2w-release/app.bin`: OTA upload
- `build-pico2w-release/app.elf`: debug symbols
- `build-pico2w-release/app.hex`: alternate flashing format

## Flashing And OTA

USB first install or rollback:

1. Disconnect motor power.
2. Hold BOOTSEL while plugging in USB.
3. Copy `app.uf2` to the `RPI-RP2` drive.
4. Reboot and verify the firmware version.

OTA after the device is already running OTA-capable firmware:

```powershell
python tools/ota_upload.py --host <device-ip-or-hostname> --bin build-pico2w-release/app.bin --apply
```

Use USB rollback if OTA fails, Wi-Fi is unstable, or the version cannot be verified after reboot.

## Repository Map

- `.github/workflows/cmake.yml`: GitHub Actions firmware build for Pico 2 W / RP2350.
- `docs/`: GitHub Pages-ready public beta documentation.
- `firmware_release_history/`: public-safe changelog, issue ledger, and release notes.
- `library/`: embedded libraries and submodule content.
- `manuals/`: original flashing, wireless, and scale setup material.
- `resources/`: screenshots and diagrams used by documentation.
- `scripts/`: helper scripts for firmware development.
- `src/`: firmware source, REST endpoints, web portal, drivers, charge mode, OTA, and AI model code.
- `targets/`: board and hardware target support.
- `tests/`: local tests and validation material.
- `tools/ota_upload.py`: Wi-Fi OTA uploader.
- `RELEASE_VERSION`: firmware release identity consumed by version generation.

## Safety Notes

- This is beta firmware, not a published stable release.
- Confirm the selected profile before charging.
- Confirm the scale is stable and calibrated before every run.
- Verify charges independently.
- Keep the device attended while motors are active.
- Keep a known-good rollback UF2 available.

## License

This project retains the upstream OpenTrickler firmware license. See [LICENSE](LICENSE).
