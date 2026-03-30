# Drivebase subsystem

## Overview

The drivebase is the robot's drivetrain — the set of wheels and motors that move
the robot around the field. This robot uses a
[swerve drive](../../GLOSSARY.md#swerve-drive), which means each wheel module
can independently steer (rotate) and drive (spin). That gives the robot
[holonomic](../../GLOSSARY.md#holonomic) motion: it can slide sideways, drive
diagonally, or spin in place, all at the same time.

The drivebase subsystem owns [odometry](../../GLOSSARY.md#odometry), provides
[field-centric](../../GLOSSARY.md#field-centric-control) driving for teleop, and
exposes pose-targeting helpers for autonomous commands.

## How it works

### Field-centric control

When a driver pushes the joystick forward, they expect the robot to move toward
the far end of the field — not "forward" relative to whatever direction the
robot happens to be facing. This is
[field-centric control](../../GLOSSARY.md#field-centric-control).

The drivebase reads the robot's heading from its
[gyroscope](../../GLOSSARY.md#gyroscope) and rotates the joystick input so the
chassis speeds are always relative to the field. A small
[deadband](../../GLOSSARY.md#deadband) is applied to joystick axes to filter out
stick drift.

### Swerve modules and YAGSL

Each [swerve](../../GLOSSARY.md#swerve-drive) module has a drive motor (spins
the wheel) and a steer motor (rotates the module). The low-level control loops
that coordinate all four modules are handled by [YAGSL](../../GLOSSARY.md#yagsl)
(Yet Another Generic Swerve Library). YAGSL reads module configuration files
from `src/main/deploy/swerve/` and manages kinematics, motor
[PID](../../GLOSSARY.md#pid) loops, and [encoder](../../GLOSSARY.md#encoder)
feedback internally.

The drivebase subsystem talks to YAGSL through the `DriveBaseIO` interface,
which follows the [AdvantageKit IO](../../GLOSSARY.md#io-inputoutput) pattern.
`DriveBaseIOYagsl` pulls pose, gyro, and module states from the active
`SwerveDrive` object for logging and telemetry.

### Odometry

[Odometry](../../GLOSSARY.md#odometry) combines wheel
[encoder](../../GLOSSARY.md#encoder) readings with
[gyroscope](../../GLOSSARY.md#gyroscope) data to estimate the robot's
[pose](../../GLOSSARY.md#pose) on the field. YAGSL's internal
`SwerveDrivePoseEstimator` runs the
[Kalman filter](../../GLOSSARY.md#kalman-filter) that fuses odometry with vision
corrections forwarded by the [robot state subsystem](../robotstate/README.md).

The drivebase exposes `getFusedPose()` for drivebase-internal use (commands,
AutoBuilder, heading PID). External subsystems should use
`RobotPoseSubsystem.getEstimatedPose()` for consistency.
`addVisionMeasurement()` lets the robot pose subsystem inject camera-based
corrections into YAGSL's pose estimator.

### Autonomous path following

For autonomous mode, the drivebase integrates with PathPlanner through
`PathPlannerCommandFactory`. PathPlanner generates trajectories from deployed
`.path` files and feeds [holonomic](../../GLOSSARY.md#holonomic) chassis speed
commands back into the drivebase. The drivebase's
[PID](../../GLOSSARY.md#pid)-based pose-targeting helpers (translation PID and
heading PID) keep the robot tracking the planned path.

## Configuration

### Deploy files

Module geometry, motor types, and low-level PIDF gains live in the swerve deploy
folder at `src/main/deploy/swerve/`. These files are consumed directly by YAGSL
and include controller properties, module definitions, and PIDF configurations.

### Subsystem tunables

Higher-level settings live in `subsystems.json` under `driveBaseSubsystem`. All
values are [tunable](../../GLOSSARY.md#tunable) through SmartDashboard.

| Setting                               | Units    | Purpose                                                          |
| ------------------------------------- | -------- | ---------------------------------------------------------------- |
| `maximumSpeedMetersPerSecond`         | m/s      | Top translational speed                                          |
| `maximumAngularSpeedRadiansPerSecond` | rad/s    | Top rotational speed                                             |
| `translationPidKp`, `Ki`, `Kd`        | unitless | [PID](../../GLOSSARY.md#pid) for autonomous translation tracking |
| `rotationPidKp`, `Ki`, `Kd`           | unitless | [PID](../../GLOSSARY.md#pid) for autonomous heading tracking     |
| `translationToleranceMeters`          | meters   | Position tolerance for pose targeting                            |
| `rotationToleranceRadians`            | radians  | Heading tolerance for pose targeting                             |
| `headingCorrectionEnabled`            | boolean  | Toggles YAGSL passive heading drift correction                   |
| `joystickTranslationScale`            | 0.0–1.0  | Scales joystick sensitivity                                      |

Heading hold [PID](../../GLOSSARY.md#pid) gains (`p`, `i`, `d`) are configured
in `controllerproperties.json` inside the swerve deploy folder (e.g.,
`src/main/deploy/swerve/controllerproperties.json`). These gains are shared
between the explicit heading hold controller and YAGSL's passive heading
correction feature, giving a single source of truth for heading tuning.

## Calibration and tuning guide

This section walks through every calibration step needed to get smooth, accurate
swerve behavior. Work through them in order — each step builds on the one before
it. Keep the robot on blocks (wheels off the ground) for Steps 1–6 so you can
spin modules safely.

### Prerequisites

Before you start, make sure you have:

- The robot elevated on blocks so all four wheels spin freely.
- A laptop connected to the robot via USB or radio.
- Driver Station open and the robot enabled in teleop.
- AdvantageScope connected for telemetry.
- REV Hardware Client or Phoenix Tuner available to verify CAN device IDs.
- The deploy directory identified: `src/main/deploy/swerve/`.

### Step 1: Verify module identity and CAN IDs

Each module JSON file in `src/main/deploy/swerve/modules/` must match the
physical corner it describes. If a file is assigned to the wrong corner, the
robot will spin out of control or drive sideways.

Open REV Hardware Client (for Spark Flex) and Phoenix Tuner (for CANcoders and
the Pigeon 2) and confirm that every CAN ID in the JSON matches the hardware.

| Module      | File              | Drive CAN | Angle CAN | Encoder CAN |
| ----------- | ----------------- | --------- | --------- | ----------- |
| Front Left  | `frontleft.json`  | 12        | 10        | 11          |
| Front Right | `frontright.json` | 42        | 40        | 41          |
| Back Left   | `backleft.json`   | 22        | 20        | 21          |
| Back Right  | `backright.json`  | 32        | 30        | 31          |

The Pigeon 2 IMU is CAN ID **8** (set in `swervedrive.json`).

If a motor blinks but the wrong module moves, the CAN IDs are swapped between
files. Fix them in the JSON before continuing.

### Step 2: Set absolute encoder offsets

The absolute [encoder](../../GLOSSARY.md#encoder) offset tells YAGSL where "zero
degrees" is for each module. Getting this wrong makes the wheels point in random
directions at startup.

1. Orient all four modules so the wheel bevels face **left** when viewed from
   above. This is the YAGSL-defined "forward" position (see the
   [YAGSL configuration guide](https://docs.yagsl.com/configuring-yagsl/configuration)).
2. Deploy the code and enable the robot in teleop.
3. In NetworkTables (via AdvantageScope or SmartDashboard), read the value of
   `swerve/modules/<moduleName>/Raw Absolute Encoder` for each module.
4. Enter each value as `absoluteEncoderOffset` (in degrees) in the corresponding
   module JSON file.
5. Redeploy and verify the wheels all point forward at startup.

> **Odometry drives backward?** If real-life forward motion shows as backward in
> odometry (but rotation is correct), add `180` to every module's
> `absoluteEncoderOffset` and redeploy.

### Step 3: Verify motor and IMU inversions

The goal is to make every sensor value increase when rotated
**counterclockwise** (CCW-positive), which is the WPILib convention. Run each
check with the robot on blocks.

**Steer (angle) motors:**

1. Spin each module counterclockwise from a top-down view.
2. Watch `swerve/modules/<moduleName>/Raw Angle Encoder` in NetworkTables.
3. If the value **decreases**, set `inverted.angle` to `true` in that module's
   JSON.

**Drive motors:**

1. Spin each wheel counterclockwise (looking at the wheel from the outside).
2. Watch `swerve/modules/<moduleName>/Raw Drive Encoder`.
3. If the value **decreases**, set `inverted.drive` to `true` in that module's
   JSON.

**IMU (Pigeon 2):**

1. Rotate the entire robot counterclockwise (top-down view).
2. Watch `Raw IMU Yaw` in NetworkTables.
3. If it **decreases**, set `invertedIMU` to `true` in `swervedrive.json`.

Our current settings are `inverted.drive = true`, `inverted.angle = true`, and
`invertedIMU = false`. Only change these if you replace hardware or swap module
positions.

### Step 4: Verify conversion factors

Conversion factors tell YAGSL how to translate motor rotations into wheel angle
(degrees) and distance (meters). Wrong values cause incorrect speeds and
[odometry](../../GLOSSARY.md#odometry) drift.

Default values live in `modules/physicalproperties.json` and apply to every
module. Individual modules can override them with a `conversionFactors` block in
their own JSON file.

| Property            | Where to find it                                                 | Current default  |
| ------------------- | ---------------------------------------------------------------- | ---------------- |
| Angle gear ratio    | `conversionFactors.angle.gearRatio` in `physicalproperties.json` | 21.43 (SDS MK4i) |
| Drive gear ratio    | `conversionFactors.drive.gearRatio` in `physicalproperties.json` | 5.9              |
| Wheel diameter (in) | `conversionFactors.drive.diameter` in `physicalproperties.json`  | 4                |

Our front modules override the angle gear ratio to **18.75** (SDS MK4n), while
the back modules inherit the default **21.43** (SDS MK4i). If you swap module
hardware, update these values to match. YAGSL maintains a
[standard conversion factors](https://docs.yagsl.com/configuring-yagsl/standard-conversion-factors)
page with values for common COTS modules.

**How to verify:** Drive a known distance (e.g., tape measure 3 meters on the
floor), then compare the odometry position in AdvantageScope. If the reported
distance is off by more than 5%, your gear ratio or wheel diameter is wrong.

### Step 5: Tune per-module angle (steer) PID

The angle [PID](../../GLOSSARY.md#pid) controls how quickly and smoothly each
module rotates to its target heading. These gains go in each module JSON under
the `angle` motor's `p`, `i`, and `d` fields. Per-module values override the
defaults in `pidfproperties.json`.

> **Important:** YAGSL enables PID wrapping on steer motors at 0° and 360°. To
> avoid masking issues at the wrap point, test with **left/right translation**
> (strafe), not pure rotation.

**Tuning procedure** (from the
[YAGSL tuning guide](https://docs.yagsl.com/configuring-yagsl/how-to-tune-pidf)):

1. Set `p`, `i`, and `d` all to **0**.
2. Increase `p` until the module starts oscillating around its target angle.
3. Increase `d` until the oscillation dampens and the module settles quickly
   without jitter.
4. Leave `i` at **0** unless there is a persistent steady-state offset that `p`
   cannot fix.
5. Repeat for each module individually — gains can vary between corners because
   of friction, belt tension, or mechanical differences.

Current tuned values (for reference):

| Module      | Angle P | Angle D |
| ----------- | ------- | ------- |
| Front Left  | 1.763   | 38.795  |
| Front Right | 1.8952  | 56.839  |
| Back Left   | 1.7059  | 17.673  |
| Back Right  | 1.7059  | 17.673  |

The front and back values differ significantly because the module hardware has
different gear ratios (18.75 vs 21.43). Re-tune whenever you replace a motor,
belt, or module.

### Step 6: Tune per-module drive PID

The drive [PID](../../GLOSSARY.md#pid) controls each wheel's velocity tracking.
These gains go in each module JSON under the `drive` motor's `p`, `i`, and `d`
fields.

**Tuning procedure:**

1. Start with the YAGSL default for Spark Flex / NEO Vortex motors:
   `p = 0.0020645` (from `pidfproperties.json`).
2. Command a moderate forward speed and watch the wheel velocities in
   AdvantageScope.
3. If the wheels are slow to reach target speed, increase `p`.
4. If the wheels oscillate or jerk, decrease `p`.
5. Leave `d` at **0** for velocity control — derivative gain on a noisy velocity
   signal usually hurts more than it helps.
6. Leave `i` at **0** unless there is a consistent steady-state velocity error.

Current tuned values:

| Module      | Drive P  |
| ----------- | -------- |
| Front Left  | 0.033935 |
| Front Right | 0.07003  |
| Back Left   | 5.36e-5  |
| Back Right  | 5.36e-5  |

The front and back modules have very different gains. This may reflect hardware
differences or an incomplete tuning pass. Re-tune each module after any
mechanical change.

### Step 7: Run SysId for drive and angle motors (optional)

WPILib's System Identification (SysId) tool can characterize the drive and steer
motors to produce feedback gains. This step is optional but recommended for
competition-level accuracy.

#### What SysId results are used for

YAGSL only uses **kP** for drive motors (velocity P-loop) and **kP + kD** for
angle motors (position PD-loop). The feedforward values (kS, kV, kA) produced by
SysId are **not wired into YAGSL's module control pipeline** — they serve as
diagnostic data for comparing module health and flagging mechanical issues
(e.g., a module with a significantly different kV likely has a problem).

#### SysId configuration

All SysId timing is read from the `sysId` block in `subsystems.json` under
`driveBaseSubsystem`. The same config applies to both grouped and per-module
tests:

| Parameter                   | Default | Purpose                                |
| --------------------------- | ------- | -------------------------------------- |
| `rampRateVoltsPerSecond`    | 1.0     | Quasistatic voltage ramp rate (V/s)    |
| `stepVoltage`               | 6.0     | Dynamic test step voltage (V)          |
| `delaySeconds`              | 10.0    | Pause between test phases (s)          |
| `quasistaticTimeoutSeconds` | 40.0    | Duration of each quasistatic phase (s) |
| `dynamicTimeoutSeconds`     | 4.0     | Duration of each dynamic phase (s)     |

#### Grouped SysId (all motors at once)

The `DriveBaseSubsystemCommandFactory` provides two commands that test all four
motors of the same type simultaneously:

- `createDriveSysIdCommand()` — exercises all four drive motors.
- `createAngleSysIdCommand()` — exercises all four steer motors.

These use YAGSL's built-in `SwerveDriveTest` routines, which handle the
quasistatic ramp, dynamic step, and data logging automatically. In tuning mode,
the driver Y button runs angle SysId followed by drive SysId in sequence.

#### Per-module SysId (one motor at a time)

Per-module SysId isolates a single motor for characterization while holding all
other motors at zero voltage. This is valuable because each module may have
different friction, gear mesh, or wiring characteristics.

The factory provides two parameterized methods:

- `createDriveSysIdCommandForModule(int moduleIndex)` — exercises one drive
  motor.
- `createAngleSysIdCommandForModule(int moduleIndex)` — exercises one angle
  motor.

Module index constants are defined on `DriveBaseSubsystemCommandFactory`:
`MODULE_FRONT_LEFT (0)`, `MODULE_FRONT_RIGHT (1)`, `MODULE_BACK_LEFT (2)`,
`MODULE_BACK_RIGHT (3)`.

In tuning mode (`tuningEnabled = true`), the operator controller is mapped to
per-module SysId:

| Button        | Motor Type | Module      |
| ------------- | ---------- | ----------- |
| A             | Drive      | Front Left  |
| B             | Drive      | Front Right |
| X             | Drive      | Back Left   |
| Y             | Drive      | Back Right  |
| Left Bumper   | Angle      | Front Left  |
| Right Bumper  | Angle      | Front Right |
| Left Trigger  | Angle      | Back Left   |
| Right Trigger | Angle      | Back Right  |

**To run per-module SysId:**

1. Set `tuningEnabled` to `true` in `subsystems.json` and restart the robot
   code.
2. Place the robot on blocks with wheels free to spin.
3. Enable teleop and press the operator button for the motor you want to test.
   Hold the button — it runs all four test phases automatically. Release to
   cancel early.
4. Repeat for each motor.
5. Disable the robot and pull the WPILog file.
6. Open WPILib's SysId analysis tool, load the log, and read kP from the results
   for each drive motor.
7. Enter the kP values into each module JSON file's `"drive"` → `"p"` field. Use
   kS/kV/kA to compare modules and flag outliers.

> **Note:** The YAGSL swerve SysId workflow is different from the project's
> `SysIdHelper`-based workflow used by other subsystems (turret, shooter, etc.).
> The ÷2π correction described in the
> [SysId tuning guide](../../shared/subsystems/SYSID_GUIDE.md) does **not**
> apply here because YAGSL handles its own unit conversions internally.

### Step 8: Tune heading PID

The heading [PID](../../GLOSSARY.md#pid) keeps the robot pointing at a target
angle during teleop (heading hold) and autonomous (path rotation tracking). It
acts on the [gyroscope](../../GLOSSARY.md#gyroscope) yaw reading.

Gains are set in two places:

- **Deploy file:** `controllerproperties.json` → `heading.p`, `heading.i`,
  `heading.d` (loaded at startup by YAGSL).
- **Runtime tunables:** `subsystems.json` → `headingKp`, `headingKi`,
  `headingKd` (applied by `DriveBaseSubsystem.refreshTunables()` every cycle
  when not connected to FMS).

The runtime tunables override the deploy file values, so tune via SmartDashboard
during practice and then copy the final values back into both files.

**Tuning procedure:**

1. Set `headingKi` and `headingKd` to **0**. Start `headingKp` at **0.5**.
2. Enable teleop and drive forward. Release the sticks — the robot should hold
   its heading.
3. Push the robot off-angle by hand. It should rotate back to the original
   heading.
4. If the correction is sluggish, increase `headingKp`.
5. If the robot oscillates or overshoots the target angle, decrease `headingKp`
   or add a small `headingKd` (start around 0.01).
6. Adjust `rotationToleranceDegrees` (default 2.0°) if the robot hunts back and
   forth near the target — a wider tolerance allows it to settle sooner.

Current values: `headingKp = 0.85`, `headingKi = 0`, `headingKd = 0.02`.

### Step 9: Tune path-following PID

PathPlanner uses two separate [PID](../../GLOSSARY.md#pid) controllers to keep
the robot on its planned trajectory during autonomous:

- **Translation PID** (`pathTranslationKp/Ki/Kd`) — corrects position error in
  meters.
- **Rotation PID** (`pathRotationKp/Ki/Kd`) — corrects heading error in radians.

Both are set in `subsystems.json` and are tunable via SmartDashboard.

**Tuning procedure:**

1. Create a simple autonomous path (a straight line or gentle curve).
2. Deploy, run the auto, and overlay the planned vs. actual path in
   AdvantageScope.
3. If the robot cuts corners or lags behind the path, increase the translation
   `Kp`.
4. If the robot oscillates laterally along the path, decrease the translation
   `Kp` or add a small `Kd`.
5. Repeat the same process for the rotation controller by watching heading error
   during turns.

Current values: `pathTranslationKp = 5`, `pathRotationKp = 5` (all Ki and Kd are
0).

### Step 10: Validate on the field

With all gains tuned on blocks, put the robot on the ground for a final
checkout.

1. **Translate in all four directions** (forward, backward, left, right). The
   robot should move cleanly without pulling to one side.
2. **Strafe diagonally.** Verify the path is straight, not curved.
3. **Spin in place.** The robot should rotate smoothly without translating.
4. **Drive and rotate simultaneously.** Field-centric control should feel
   intuitive.
5. **Check odometry.** Drive a known distance and compare the AdvantageScope
   pose to the real position. Heading should also match.
6. **Run a known autonomous path.** Compare planned vs. actual trajectory in
   AdvantageScope.

**Troubleshooting quick reference:**

| Symptom                                      | Likely cause                                     |
| -------------------------------------------- | ------------------------------------------------ |
| Robot drives sideways or at an angle         | Module CAN IDs swapped between JSON files        |
| Wheels point in random directions            | Absolute encoder offsets are wrong (Step 2)      |
| Robot spins uncontrollably                   | Motor inversions are wrong (Step 3)              |
| Odometry drifts over time                    | Gear ratios or wheel diameter incorrect (Step 4) |
| Odometry shows backward when driving forward | Add 180° to all encoder offsets (Step 2)         |
| Wheels stutter or vibrate                    | Angle or drive PID too high (Steps 5–6)          |
| Wheels respond slowly                        | Angle or drive PID too low (Steps 5–6)           |
| Robot overshoots heading target              | Heading Kp too high or Kd too low (Step 8)       |
| Auto path tracking is sloppy                 | Path-following PID needs tuning (Step 9)         |

### Configuration file quick reference

| Concern                    | File                                     | Key fields                                                               |
| -------------------------- | ---------------------------------------- | ------------------------------------------------------------------------ |
| IMU type and CAN ID        | `swerve/swervedrive.json`                | `imu.type`, `imu.id`, `invertedIMU`                                      |
| Module CAN IDs             | `swerve/modules/<module>.json`           | `drive.id`, `angle.id`, `encoder.id`                                     |
| Absolute encoder offset    | `swerve/modules/<module>.json`           | `absoluteEncoderOffset` (degrees)                                        |
| Motor inversions           | `swerve/modules/<module>.json`           | `inverted.drive`, `inverted.angle`                                       |
| Gear ratios / wheel size   | `swerve/modules/physicalproperties.json` | `conversionFactors.angle.gearRatio`, `.drive.*`                          |
| Per-module gear overrides  | `swerve/modules/<module>.json`           | `conversionFactors` (overrides physicalproperties)                       |
| Default motor PIDF         | `swerve/modules/pidfproperties.json`     | `drive.p/i/d/f`, `angle.p/i/d/f`                                         |
| Per-module motor PIDF      | `swerve/modules/<module>.json`           | `drive.p/i/d`, `angle.p/i/d`                                             |
| Heading PID (deploy)       | `swerve/controllerproperties.json`       | `heading.p/i/d`                                                          |
| Heading PID (runtime)      | `subsystems.json` → `driveBaseSubsystem` | `headingKp`, `headingKi`, `headingKd`                                    |
| Path-following PID         | `subsystems.json` → `driveBaseSubsystem` | `pathTranslationKp/Ki/Kd`, `pathRotationKp/Ki/Kd`                        |
| Max speeds                 | `subsystems.json` → `driveBaseSubsystem` | `maximumLinearSpeedFeetPerSecond`, `maximumAngularSpeedDegreesPerSecond` |
| Current limits / ramp rate | `swerve/modules/physicalproperties.json` | `currentLimit.*`, `rampRate.*`                                           |

## Code structure

| File                                             | Purpose                                                                                                   |
| ------------------------------------------------ | --------------------------------------------------------------------------------------------------------- |
| `DriveBaseSubsystem.java`                        | Owns odometry, field-relative driving, holonomic pose chasing, joystick scaling, and AdvantageKit logging |
| `commands/MoveFieldManualCommand.java`           | Applies field-relative chassis speeds from joystick input                                                 |
| `commands/DriveBaseSubsystemCommandFactory.java` | Wires driver axes, sets the default manual command, and exposes SysId commands via YAGSL                  |
| `commands/PathPlannerCommandFactory.java`        | Builds autonomous path-following commands from deployed `.auto` files                                     |
| `config/DriveBaseSubsystemConfig.java`           | Tunables for max speed, PID gains, tolerances, and joystick sensitivity                                   |
| `io/DriveBaseIO.java`                            | AdvantageKit IO interface — defines logged inputs for pose, gyro, and module states                       |
| `io/DriveBaseIOYagsl.java`                       | YAGSL hardware implementation — pulls data from the active `SwerveDrive`                                  |

## Status / TODO

### Done

- Field-centric swerve driving with deadband and scaling.
- Odometry via YAGSL's `SwerveDrivePoseEstimator`.
- Vision measurement injection for pose fusion.
- PathPlanner integration for autonomous paths.
- SysId support via YAGSL helpers.
- Sim-friendly fallback when subsystem is disabled.
- Calibration and tuning guide added to this README.

### TODO

- Add more autonomous poses and driver aids as needed.
