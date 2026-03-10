# Turret subsystem

## Overview

The turret is a single-axis rotating mount that turns left and right so the
shooter can aim without moving the entire robot. It uses one Spark MAX motor
controller driving through a gear reduction for smooth, precise positioning. The
turret is the canonical example of the
[set-and-seek](../../shared/README.md#the-set-and-seek-pattern) pattern — if you
are building a new profiled mechanism, read this README first.

## How it works

The turret follows the [set-and-seek](../../GLOSSARY.md#set-and-seek) pattern. A
command sets a target angle in degrees, and the subsystem plans a smooth path to
that angle using a
[trapezoidal motion profile](../../GLOSSARY.md#trapezoidal-motion-profile). Each
code cycle, the profile steps forward and a
[profiled PID](../../GLOSSARY.md#profiled-pid) controller with
[feedforward](../../GLOSSARY.md#feedforward) computes the motor voltage needed
to track the moving setpoint.

### Step-by-step move

1. A command calls `setTarget(degrees)`.
2. The base class clamps the target to the configured min/max limits.
3. The value is converted from degrees to radians for internal math.
4. Each cycle, `seekTarget()` advances the
   [trapezoidal profile](../../GLOSSARY.md#trapezoidal-motion-profile) by one
   step.
5. [PID](../../GLOSSARY.md#pid) computes a correction from the difference
   between the setpoint and the measured position.
6. [Feedforward](../../GLOSSARY.md#feedforward) estimates the voltage needed for
   the planned motion (using [kS, kV, kA](../../GLOSSARY.md#feedforward-gains)).
7. The combined PID + feedforward output is sent to the motor.
8. The command finishes when `isProfileSettled()` reports that position and
   velocity are within tolerance of the goal.

### Trapezoidal profile (ASCII sketch)

Velocity vs. time for a trapezoidal move:

```
velocity
  ^            ____________  <- cruise at max velocity
  |           /            \
  |          /              \
  |         /                \
  |________/                  \________  -> time
           accel            decel
```

If the target is close, the profile ramps up and back down without ever reaching
cruise speed — the trapezoid becomes a triangle.

### How commands drive the turret

- `MoveTurretToAngleCommand` extends `AbstractSetAndSeekCommand`.
- On initialize, the command calls `setTarget()` once using a supplier.
- During execute, the command calls `seekTarget()` every loop to advance the
  profile.
- The command finishes when `isProfileSettled()` returns true.
- If the command is interrupted, the base command schedules a settle command so
  the turret decelerates safely.
- `TrackFieldTargetCommand` continuously recalculates a field-relative target
  each cycle and feeds it to `setTarget()`, allowing the turret to track a
  moving or alliance-relative position.

### Positioning and units

We use degrees in the public API because it is easier for humans to read and
think about. WPILib math uses radians, so we convert once when a target is set.

The turret faces the **rear** of the robot. The `turretZeroOffsetDegrees` config
field is set to `180` to account for this — when the turret is at its mechanical
zero, it points backward. The field-target tracking math subtracts this offset
from the robot-relative angle to convert into turret-relative coordinates, so
that "aim at a field position" automatically compensates for the rear-facing
mount. All setpoint limits (`minimumSetpointDegrees`, `maximumSetpointDegrees`)
and soft limits are relative to the turret's own zero, not to robot-forward.

- Commands call `setTarget(double)` with a target in degrees.
- `AbstractSetAndSeekSubsystem` clamps the target between the configured minimum
  and maximum and then converts to radians for the controller.
- `TurretMotor` configures the Spark MAX [encoder](../../GLOSSARY.md#encoder)
  conversion factors so the encoder reports turret radians, not raw motor
  rotations.
- [AdvantageKit](../../GLOSSARY.md#advantagekit) logs are recorded in degrees in
  the base class for clarity.

### Gear setup and ratio math

The Spark MAX uses the internal relative encoder, which measures motor
rotations. The turret itself moves much slower because of the gear reduction. We
must scale the [encoder](../../GLOSSARY.md#encoder) using the full
[gear ratio](../../GLOSSARY.md#gear-ratio).

We store the ratio as **motor rotations per turret rotation**:

```
motor rotations per turret rotation = gearbox ratio × (driven teeth / driver teeth)
```

Example: for a 5 : 1 gearbox with a 16-tooth gear driving a 152-tooth gear:

```
ratio = 5 × (152 / 16) = 47.5
```

That ratio is stored in `subsystems.json` as
`turretSubsystem.turretMotorConfig.motorRotationsPerMechanismRotation`. The
motor wrapper converts it to **radians per motor rotation** so the encoder
reports true turret radians.

If the turret moves twice as far as expected, the gear ratio is usually off by a
factor of two. Check every gear stage and tooth count to confirm the real ratio.

## Configuration

All turret settings live in `src/main/deploy/subsystems.json` (or the sim/test
variants) under the `turretSubsystem` key. Values are
[tunable](../../GLOSSARY.md#tunable) — you can adjust them on the fly through
SmartDashboard without redeploying.

### Key tunables

| Setting                                      | Units      | Purpose                                                         |
| -------------------------------------------- | ---------- | --------------------------------------------------------------- |
| `minimumSetpointDegrees`                     | degrees    | Reverse rotation limit                                          |
| `maximumSetpointDegrees`                     | degrees    | Forward rotation limit                                          |
| `maximumVelocityDegreesPerSecond`            | deg/s      | Profile cruise speed                                            |
| `maximumAccelerationDegreesPerSecondSquared` | deg/s²     | Profile ramp rate                                               |
| `positionToleranceDegrees`                   | degrees    | How close is "close enough"                                     |
| `velocityToleranceDegreesPerSecond`          | deg/s      | How slow is "stopped enough"                                    |
| `kP`, `kI`, `kD`                             | unitless   | [PID gains](../../GLOSSARY.md#pid-gains)                        |
| `kS`                                         | volts      | [Static feedforward](../../GLOSSARY.md#feedforward-gains)       |
| `kV`                                         | V/(deg/s)  | [Velocity feedforward](../../GLOSSARY.md#feedforward-gains)     |
| `kA`                                         | V/(deg/s²) | [Acceleration feedforward](../../GLOSSARY.md#feedforward-gains) |

Motor-specific settings ([CAN](../../GLOSSARY.md#can-bus) ID, inversion, current
limits, [gear ratio](../../GLOSSARY.md#gear-ratio)) live under
`turretSubsystem.turretMotorConfig`.

### Tuning checklist

1. Confirm the [gear ratio](../../GLOSSARY.md#gear-ratio) first — wrong ratios
   cause every other value to misbehave.
2. Choose safe max speed and [acceleration](../../GLOSSARY.md#acceleration)
   values before tuning gains.
3. Use [SysId](../../GLOSSARY.md#sysid) to measure kS, kV, and kA so feedforward
   is close from the start.
4. Increase kP until tracking is strong but not oscillating.
5. Add a small kD if you see [overshoot](../../GLOSSARY.md#overshoot).
6. Keep test moves simple and repeatable so changes are easy to compare.

## Code structure

| File                                          | Purpose                                                                                             |
| --------------------------------------------- | --------------------------------------------------------------------------------------------------- |
| `TurretSubsystem.java`                        | Extends `AbstractSetAndSeekSubsystem`; builds either `TurretMotor` (real) or `TurretSimMotor` (sim) |
| `commands/MoveTurretToAngleCommand.java`      | Profiled move command — extends `AbstractSetAndSeekCommand`                                         |
| `commands/TrackFieldTargetCommand.java`       | Continuous field-relative tracking command                                                          |
| `commands/TurretSubsystemCommandFactory.java` | Factory that builds commands for `RobotContainer` wiring                                            |
| `config/TurretSubsystemConfig.java`           | Extends `AbstractSetAndSeekSubsystemConfig`; adds turret-specific fields                            |
| `config/TurretMotorConfig.java`               | Motor-level config (CAN ID, gear ratio, current limits)                                             |
| `devices/TurretMotor.java`                    | Real hardware motor wrapper — extends `AbstractMotor`                                               |
| `devices/TurretSimMotor.java`                 | Simulation motor wrapper — extends `AbstractSimMotor`                                               |

## Status / TODO

### Done

- Full set-and-seek profiled motion with PID + feedforward.
- SysId characterization commands via the command factory.
- Simulation support via `TurretSimMotor`.
- Field-target tracking command for continuous aiming.

### TODO

- Revisit aiming presets and vision integration once shooter testing begins.
