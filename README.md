# OpenTrickler ML Firmware

Private beta firmware for the OpenTrickler RP2040/RP2350 controller, with profile-aware flow characterization, machine calibration, runtime learning, steering controls, REST status endpoints, and OTA staging.

> Early beta: this firmware controls reloading equipment. Verify every charge with a calibrated scale, supervise all operation, and keep a known-good UF2 rollback image available.

## Project Status

- Current beta identity: `2026.06.16-beta.8`
- Primary target: Raspberry Pi Pico 2 W / RP2350
- Also builds for: Raspberry Pi Pico W / RP2040
- Repository status: private beta, not ready for broad public release
- Documentation site: [docs/index.html](docs/index.html)

## What Is Different From Original OpenTrickler Firmware

This branch is based on the OpenTrickler RP2040 controller firmware, but adds an adaptive tuning layer around the existing charge workflow.

- Adds AI flow characterization for coarse, trim, fine, and micro-recovery behavior.
- Adds machine calibration for scale response, settle timing, open-loop flow, and tail behavior.
- Saves learned models per profile instead of relying only on fixed motor settings.
- Records runtime observations during normal throws and uses them to refine the saved model.
- Adds user steering actions: Faster, Safer, Fine Finish Faster, Bulk Closer, and Undo Last.
- Adds two-stage final recovery: `fine_recover` for larger underthrows and `micro_heal` near target.
- Adds REST endpoints for AI status, model history, steering, charge state, system information, and OTA staging.
- Adds a web-portal AI tuning panel for characterization, calibration, save/apply, and steering.
- Adds Pico 2 W / RP2350 support while keeping Pico W / RP2040 support.
- Fixes and hardens several AI telemetry and recovery behaviors found during beta testing.

## What Is Intentionally Not In This Repository

The GitHub repository is kept clean for firmware review. Local test material stays local.

- No field data collector application or packaged collector zips.
- No raw flight logs, charge CSV files, REST captures, or private test sessions.
- No generated firmware binaries committed under release history.
- No binary comparison dumps or reverse-analysis artifacts.
- No personal LAN IP addresses in user-facing documentation.

Build outputs and private test evidence should be shared through private release assets or support channels only when needed.

## Feature Overview

### Profile-Aware AI Model

Each profile can hold its own learned model. Use separate profiles for different powders, tube setups, scale behavior, or hardware changes. A model that works well for one powder/setup should not be assumed to be correct for another.

### Characterization

Characterization measures how powder moves through the current machine and profile. It samples coarse and fine behavior, estimates flow and tail, builds a finish plan, and ends in a `ready_to_save` state when the working model can be applied to the selected profile.

### Machine Calibration

Machine calibration runs after a characterization model exists. It measures scale/sample timing, response delay, settle behavior, coarse tail, fine tail, open-loop flow, and handoff margins so production throws can make safer decisions.

### Runtime Learning

During normal production, the firmware observes final error, phase timing, stop weights, recovery motor time, and stall count. Valid observations refine the saved profile model over time.

### Steering

The web portal can gently bias a saved model without rerunning characterization:

- `Faster`: allows more aggressive timing where the model has confidence.
- `Safer`: increases conservatism and finish margin.
- `Fine Finish Faster`: nudges the fine finish phase for speed.
- `Bulk Closer`: lets coarse/bulk delivery get closer before handoff.
- `Undo Last`: restores the previous steering snapshot when available.

Use one steering step at a time, then run enough charges to observe the effect.

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
7. Watch for repeatable under/over behavior before applying steering.

## Build From Source

Install Git, CMake, Ninja, the ARM GCC toolchain, Python 3, and the Raspberry Pi Pico SDK. The included PowerShell helper expects the Pico SDK environment used by the Raspberry Pi Pico extension.

```powershell
git clone https://github.com/jeanphilippebellavance-cmd/Opentrickler_ML.git
cd Opentrickler_ML
git submodule update --init --recursive
.\configure_env.ps1
cmake -B build-pico2w-release -G Ninja -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico2_w
cmake --build build-pico2w-release --config Release
```

Pico W build:

```powershell
.\configure_env.ps1
cmake -B build-picow-release -G Ninja -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico_w
cmake --build build-picow-release --config Release
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

- `.github/workflows/cmake.yml`: GitHub Actions firmware build for Pico W and Pico 2 W.
- `docs/`: GitHub Pages-ready private beta documentation.
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
