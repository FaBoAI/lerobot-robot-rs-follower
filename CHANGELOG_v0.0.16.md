# v0.0.16

- Added verified RS05 motor-side `limit_torque` programming at startup.
- Set the requested gripper maximum to `3.0 N.m`.
- Added background decoding of RobStride type-2 operation status and type-21 fault reports.
- Added active monitoring of torque, over-current, stall-current, temperature/fault flags and feedback age.
- Added filtered q-axis current (`iqf`, `0x701A`) polling at `10 Hz`.
- Added optional configurable soft/hard current thresholds; left disabled by default until the installation's current scale is measured.
- Added `2.70 N.m` soft control, `2.95 N.m` software hard trip and `3.0 N.m` motor-side ceiling.
- Added contact hold ramp from `1.50 N.m` toward `3.00 N.m` at `2.00 N.m/s`.
- Added automatic `0.04 rad` opening backoff and close-command latch on hard torque, over-current or fault.
- Count hard trips only on unique feedback samples.
- Require fresh healthy feedback before clearing a safety latch.
- Use Type-2 position directly and run `get_angle.py` only when status feedback is stale.
- Keep free-closing Leader commands and non-gripper command scaling unchanged.
