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

The drivebase exposes `getOdometryPose()` for consumers that need the raw
odometry estimate and `addVisionMeasurement()` so the robot state subsystem can
inject camera-based corrections.

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
| `headingPidKp`, `Ki`, `Kd`            | unitless | [PID](../../GLOSSARY.md#pid) for heading hold during teleop      |
| `joystickTranslationScale`            | 0.0–1.0  | Scales joystick sensitivity                                      |

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

### TODO

- Revisit tuning and deploy configs after first on-bot drive tests.
- Add more autonomous poses and driver aids as needed.
