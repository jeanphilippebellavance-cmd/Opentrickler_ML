# OpenTrickler ML Firmware Release History

This folder is the public-safe release ledger for the private beta firmware branch.

It is intentionally documentation-only inside the Git repository. Build products, raw telemetry, private test sessions, and support captures are not committed here.

## Contents

- `CHANGELOG.md`: chronological firmware history.
- `ISSUE_LEDGER.md`: bugs, fixes, and verification status.
- `releases/<version>/RELEASE_NOTES.md`: detailed notes for one firmware beta.

## What Stays Out Of Git

- Generated `.bin`, `.uf2`, `.elf`, `.hex`, `.map`, and `.dis` files.
- Private field logs, charge samples, REST captures, and support sessions.
- Field data collector tooling and packaged collector archives.
- Binary comparison dumps, external firmware analysis, and local investigation scratch files.
- Personal LAN addresses or local filesystem paths.

Release binaries should be produced by local builds or GitHub Actions and attached as private release artifacts only when they are ready to share.

## Release Procedure

1. Update `RELEASE_VERSION`.
2. Build with `cmake --build build-pico2w-release --config Release`.
3. Confirm the generated version information.
4. Flash by USB or OTA in a supervised private test.
5. Verify the device-reported version after reboot.
6. Update the release notes with public-safe behavior, verification, known risks, and test focus.
7. Update the issue ledger with any newly verified or newly discovered behavior.

Code-complete fixes remain `awaiting-powder-test` in the issue ledger until a physical session exercises the affected behavior.
