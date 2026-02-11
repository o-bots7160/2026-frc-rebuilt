# Shooter subsystem

## Overview

The Shooter subsystem spins up [flywheels](../../GLOSSARY.md#flywheel) to launch
[FUEL](../../GLOSSARY.md#fuel) across the field or into the scoring hub. It
needs steady [RPM](../../GLOSSARY.md#rpm) so volleys stay accurate during
REBUILT matches, with close-range and long-range setpoints to cover different
scoring zones.

## How it works

The shooter uses a single flywheel wheel driven by a REV SparkMax motor
controller. A [feedforward](../../GLOSSARY.md#feedforward) term maintains the
target velocity, and a [PID](../../GLOSSARY.md#pid) controller corrects for
disturbances when the [indexer](../indexer/README.md) loads a piece and
momentarily slows the wheel.

### Units convention

The public API uses **RPM** (revolutions per minute) for all velocity values.
Internally, the subsystem converts to **radians per second** for WPILib math
(PID controllers, feedforward, trapezoidal profiles). RPM always refers to the
**mechanism (flywheel) speed**, not the motor shaft speed. The gear ratio in
`ShooterMotorConfig.motorRotationsPerMechanismRotation` handles the conversion
between the two.

### Key behaviors

- **Spin-up to target** — commands set a target RPM and the shooter ramps to it
  using a PID controller plus feedforward. An optional
  [trapezoidal motion profile](../../GLOSSARY.md#trapezoidal-motion-profile)
  limits acceleration for smoother spool-up.
- **Ready-to-fire signal** — `isReadyToFire()` returns true when the measured
  RPM is within `velocityToleranceRpm` of the target and has been stable for at
  least `settleTimeSeconds`. The [indexer](../indexer/README.md) and autonomous
  commands wait on this signal before feeding.
- **Idle** — when no shot is requested, the default `IdleShooterCommand` holds
  the flywheel at `idleVelocityRpm` so re-spool is fast.
- **Reverse** — `reverseVelocityRpm` config field allows backward spin for
  clearing stuck pieces (command not yet wired).

## Inheritance

The shooter extends the shared velocity abstraction layer:

```
AbstractSubsystem
  └─ AbstractMotorSubsystem        (motor, feedforward, SysId)
       └─ AbstractVelocitySubsystem  (PID velocity control, settle detection)
            └─ ShooterSubsystem       (concrete flywheel logic)
```

## Configuration

Settings live in `subsystems.json` under `shooterSubsystem`:

| Setting                           | Units   | Purpose                                                 |
| --------------------------------- | ------- | ------------------------------------------------------- |
| `enabled`                         | —       | Master enable flag                                      |
| `maximumVelocityRpm`              | RPM     | Maximum safe flywheel speed                             |
| `maximumAccelerationRpmPerSecond` | RPM/s   | Acceleration limit for the trapezoidal ramp             |
| `velocityToleranceRpm`            | RPM     | Window around target that counts as "ready"             |
| `settleTimeSeconds`               | seconds | How long RPM must stay in tolerance before ready signal |
| `idleVelocityRpm`                 | RPM     | Default idle speed between shots                        |
| `reverseVelocityRpm`              | RPM     | Reverse speed for clearing jams                         |
| `kS`, `kV`, `kA`                  | volts   | Feedforward gains                                       |
| `kP`, `kI`, `kD`                  | —       | PID velocity controller gains                           |

## Code structure

| File                                           | Purpose                                                          |
| ---------------------------------------------- | ---------------------------------------------------------------- |
| `ShooterSubsystem.java`                        | Concrete subsystem managing flywheel velocity and ready-to-fire  |
| `commands/ShooterSubsystemCommandFactory.java` | Factory for spin-up, idle, stop, and SysId commands              |
| `commands/SpinUpShooterCommand.java`           | Command that drives the flywheel to a target RPM                 |
| `commands/IdleShooterCommand.java`             | Default command holding idle RPM                                 |
| `config/ShooterSubsystemConfig.java`           | Configuration bundle extending `AbstractVelocitySubsystemConfig` |
| `config/ShooterMotorConfig.java`               | Motor-level config (CAN ID, inversion, current limits)           |
| `devices/ShooterMotor.java`                    | Real-hardware motor wrapper for SparkMax                         |
| `devices/ShooterSimMotor.java`                 | Simulation motor wrapper                                         |

## Status / TODO

### Done

- Subsystem, commands, config, and device wrappers implemented.
- Wired in `RobotContainer` and added to `subsystems.json`.
- Operator right-bumper test binding for spin-up at 3000 RPM.
- SysId sweep available on operator Y button.

### TODO

- Tune PID and feedforward gains on real hardware.
- Add distance-based RPM lookup via supplier.
- Wire reverse command to operator binding.
