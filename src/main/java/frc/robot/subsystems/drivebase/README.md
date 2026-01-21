# Drivebase subsystem

## Role (game/mechanical)

Runs the swerve drive so the robot moves and orients on the field. Provides
field-centric control, odometry, and pose targeting with AdvantageKit logging to
support scoring and navigation.

## Control and configuration (programming)

- Field-centric: forward on the stick is always field +X regardless of robot
  heading.
- Uses YAGSL-backed IO for module control; supports sim-only runs when disabled
  in `subsystems.json`.
- Geometry and module tuning come from deploy files under
  `src/main/deploy/swerve/` (controller properties, module definitions, PIDF
  gains).
- Converts driver inputs into field-relative chassis speeds via
  `MoveFieldManualCommand`.
- Drives toward target poses using PID when commands request autonomous-style
  aiming.

## Terminology glossary

- **Deadband**: A small input range around zero that is treated as zero to
  ignore joystick noise.
- **Holonomic**: The robot can translate in any direction while also rotating.
- **Omega**: Rotational velocity around the vertical axis (radians per second).
- **PID**: Proportional-Integral-Derivative control that corrects error over
  time.
- **Swerve**: Each wheel can steer and drive, allowing omnidirectional motion
  while rotating.
- **Theta**: The robot's heading angle around the vertical axis (radians).

## Code structure and maintenance

- Classes:
  - `DriveBaseSubsystem` owns odometry, field-relative driving, holonomic pose
    chasing, joystick deadband/scale mapping, and AdvantageKit logging. Falls
    back to a no-op IO when the subsystem is disabled in `subsystems.json`.
  - `commands/MoveFieldManualCommand` applies field-relative chassis speeds;
    `commands/DriveBaseSubsystemCommandFactory` wires driver axes, sets the
    default manual command, and exposes SysId commands via YAGSL helpers.
  - `config/DriveBaseSubsystemConfig` exposes tunables for max linear/angular
    speed and accel, translation/rotation PID gains and tolerances, heading PID,
    and joystick translation scale.
  - `io/DriveBaseIO` defines logged inputs; `io/DriveBaseIOYagsl` pulls pose,
    gyro, and module states from the active `SwerveDrive` into
    `DriveBaseIOInputsAutoLogged`.
- Reviewer notes: keep field-centric behavior and AdvantageKit logging; new
  commands should reuse subsystem helpers (mapping, pose targets, stop/lock) and
  log targets/states; maintain command names ending in `Command` and factories
  in `commands/`; preserve SysId helpers for drivetrain validation.

## TODO

- Revisit tuning and deploy configs after first on-bot drive tests; add more
  autonomous poses and driver aids as needed.
