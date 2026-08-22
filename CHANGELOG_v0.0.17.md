# v0.0.17

- Corrected RS05 private-protocol command torque scaling from the erroneous `±17 N.m` to the official `±5.5 N.m` range.
- Corrected RS05 Type-2 torque feedback scaling from `±17 N.m` to `±5.5 N.m`.
- Corrected RS05 command/status velocity scaling to `±50 rad/s`.
- Fixed the v0.0.16 gripper soft-limit issue: `2.70 N.m` had effectively triggered near `0.87 N.m` because of the incorrect feedback scale.
- Kept the requested gripper motor-side torque ceiling at `3.0 N.m`.
- Added required and verified startup programming of RS05 `limit_cur` (`0x7018`) to `6.5 A`.
- Increased `iqf` (`0x701A`) polling to `20 Hz`.
- Added default current guard thresholds: `5.5 A` soft, `6.2 A` hard, `5.0 A` release, two unique hard samples.
- Updated the contact-hold profile to start at `2.20 N.m` and ramp at `3.00 N.m/s` while leaving free-closing speed unchanged.
- Added a small torque-feedback hard-trip margin (`3.15 N.m`) above the verified motor-side `3.0 N.m` ceiling to avoid false trips at normal saturation.
- Fixed Type-21 gridlock/stall-current decoding to read fault-value bit 14.
- A zero-valued Type-21 report is no longer treated as a hard fault merely because the report frame exists.
- Added RS05 configuration validation: torque limits above `5.5 N.m` and current limits above `11 A` are rejected.
- Updated packaged/default YAML, example YAML, README and installer version check.
