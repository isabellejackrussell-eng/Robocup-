# RoboCup Robot Firmware

This is the integrated firmware described by `ROBocup_Codex_Software_Architecture.md`.
It is intentionally separate from `sensor_tests/`, which remains the source of the
known-working hardware examples.

## Safety first

The default PlatformIO environment is `sensor_test`. During startup it runs a
two-second forward/reverse motor test, so raise the robot before flashing it. It
then reads and reports sensors continuously. It does not initialise the claw.

The autonomous modes (`nav_motion_test`, `full_integration_test`, and
`competition`) boot in `IDLE`. Send `g` over USB Serial or Bluetooth before they
can command motion. Send `x` at any time to return to `IDLE` and stop the tracks.

The dedicated `motion_test` and `claw_test` environments intentionally begin their
test sequences at startup. Raise the robot or otherwise make the mechanism safe
before flashing either one.

## Build and upload

Open this `robot_firmware` directory as the PlatformIO project.

```sh
# Safe default sensor build
pio run

# Build a selected stage
pio run -e motion_test
pio run -e claw_test
pio run -e sensor_test
pio run -e sensor_claw_test
pio run -e nav_motion_test
pio run -e full_integration_test
pio run -e competition

# Upload one selected stage only after making the robot safe
pio run -e sensor_test -t upload
```

## Debug connections and commands

All tagged diagnostics are sent to both:

- USB `Serial` at 115200 baud
- Bluetooth `Serial2` at 115200 baud

Commands accepted from either connection:

| Command | Action |
|---|---|
| `h` or `?` | Print command help |
| `c` | Capture the current empty 8x8 scene as the weight-detection background |
| `p` | Print the complete 8x8 distance grid |
| `g` | Start an autonomous test from `IDLE` |
| `x` | Stop motion and return to `IDLE` |
| `r` | Request return-to-home behaviour |

Diagnostic tags show which layer is running:

- `[MOTION TEST]`: forward, reverse, stop, turn left, turn right
- `[CLAW TEST]`: release, grab, and completion state
- `[8X8]`: grid, candidate, distance, bearing, cluster size, confidence
- `[WALL RAW]`: ultrasonic and crossing-ToF measurements
- `[WALL]`: filtered wall geometry and corner flags
- `[VALIDATION]`: candidate status, wall distance, and separation
- `[CAPTURE RAW]`: both mouth sensors and trigger decision
- `[IMU]`, `[FLOW]`, `[ENCODER]`, `[POSE]`, `[HOME]`: localisation inputs/results
- `[FSM]`, `[NAV]`: current decision and commanded track speeds

## Configuration and known unknowns

All pins, addresses, thresholds, periods, and calibration values live in
`src/config/robot_config.h`.

Known values copied from the working tests:

| Device | Current value |
|---|---|
| Left/right motors | pins 0/1; right inverted |
| Encoders | left 2/3, right 4/5 |
| PMW3901 | CS pin 10 |
| Claw servo | pin 25 |
| BNO055 | address `0x28` |
| 8x8 ToF | address `0x33` |
| SX1509 | address `0x3F` |

The following deliberately remain disabled or uncalibrated until hardware tests
provide real values:

- ultrasonic trigger/echo pins;
- XSHUT mappings for the two crossing and two capture VL53L1X sensors;
- encoder ticks per millimetre;
- PMW3901 counts per millimetre and final axis orientation;
- wall/capture thresholds and mounting geometry;
- final claw angles/timing and navigation gains.

An unassigned pin is `-1`. Its driver reports `NOT CONFIGURED` and returns an
invalid reading rather than treating a missing sensor as zero distance.

## Suggested tuning order

1. Flash `motion_test`; verify the printed phase matches physical movement.
2. Flash `claw_test`; tune only the claw constants in `robot_config.h`.
3. Flash `sensor_test`; point the 8x8 sensor at an empty scene and send `c`.
4. Use `p` and the periodic tagged output to tune weight thresholds.
5. Assign the ultrasonic and VL53L1X XSHUT pins, then tune wall/corner logic.
6. Tune the two capture sensors; grabbing requires both sensors by design.
7. Measure encoder and optical-flow scale values using known physical distances.
8. Use `nav_motion_test` to compare `[POSE]` output with measured motion.
9. Use `full_integration_test`; calibrate with `c`, then explicitly start with `g`.

## Architecture boundaries

- `sensors/` reads hardware and records validity/timestamps.
- `perception/` interprets raw readings without commanding actuators.
- `localisation/` reads navigation sensors and estimates pose/home direction.
- `navigation/` creates motion commands from processed state.
- `motion/` and `claw/` perform physical actions.
- `state/` owns the robot FSM.
- `schedulers/` decides when each layer runs.
- `debug/` mirrors human-readable diagnostics to USB and Bluetooth.

Sorting, storage, deposit, colour-home confirmation, advanced mapping, and other
mechanisms listed under “Do Not Implement Yet” remain intentionally unimplemented.
