# Indexer subsystem

## Overview

The Indexer subsystem controls when [FUEL](../../GLOSSARY.md#fuel) leaves the
robot. It acts as the gate between storage ([feeder](../feeder/README.md)) and
the [shooter](../shooter/README.md), metering one piece at a time so shots can
be timed during REBUILT scoring windows. The indexer can hold a preloaded piece
for autonomous or volley sequences.

## How it works

The indexer uses a single roller driven by a REV SparkMax motor controller. A
[feedforward](../../GLOSSARY.md#feedforward) term maintains the target velocity,
and a [PID](../../GLOSSARY.md#pid) controller corrects for disturbances. The
motor can spin in both directions, enabling forward feeding and reverse
clearing.

### Units convention

The public API uses **RPM** (revolutions per minute) for all velocity values.
Internally, the subsystem converts to **radians per second** for WPILib math
(PID controllers, feedforward, trapezoidal profiles). RPM always refers to the
**mechanism speed**, not the motor shaft speed. The gear ratio in
`MotorConfig.motorRotationsPerMechanismRotation` handles the conversion between
the two.

### Key behaviors

- **Feed** — `FeedCommand` spins the roller at `feedVelocityRpm` to push a piece
  into the [shooter](../shooter/README.md) flywheels.
- **Hold** — `HoldCommand` keeps the roller stopped (0 RPM) while a piece is
  staged, preventing it from entering the shooter until conditions are met.
- **Fire-when-ready** — `createFireWhenReadyCommand()` accepts suppliers for
  shooter readiness and turret aim. It waits until both signals are true, then
  feeds. This is the primary autonomous and teleop firing path.
- **Reverse** — `ReverseIndexerCommand` spins backward at `reverseVelocityRpm`
  to clear a stuck piece.
- **Unjam** — `UnjamCommand` alternates between forward and reverse at
  configurable durations (`unjamForwardDurationSeconds` /
  `unjamReverseDurationSeconds`) to free a jammed piece.
- **Idle** — when no command is active, the default `IdleIndexerCommand` holds
  the roller at 0 RPM so the motor is not free-spinning.

## Inheritance

The indexer extends the shared velocity abstraction layer:

```
AbstractSubsystem
  └─ AbstractMotorSubsystem        (motor, feedforward, SysId)
       └─ AbstractVelocitySubsystem  (PID velocity control, settle detection)
            └─ IndexerSubsystem       (concrete roller logic)
```

## Configuration

Settings live in `subsystems.json` under `indexerSubsystem`:

| Setting                           | Units   | Purpose                                           |
| --------------------------------- | ------- | ------------------------------------------------- |
| `enabled`                         | —       | Master enable flag                                |
| `maximumVelocityRpm`              | RPM     | Maximum safe roller speed (clamp limit)           |
| `maximumAccelerationRpmPerSecond` | RPM/s   | Acceleration limit for the trapezoidal ramp       |
| `velocityToleranceRpm`            | RPM     | Window around target that counts as "at velocity" |
| `settleTimeSeconds`               | seconds | How long RPM must stay in tolerance before ready  |
| `idleVelocityRpm`                 | RPM     | Default idle speed (normally 0)                   |
| `feedVelocityRpm`                 | RPM     | Forward speed for feeding into shooter            |
| `reverseVelocityRpm`              | RPM     | Reverse speed for clearing jams                   |
| `unjamForwardDurationSeconds`     | seconds | Forward phase duration in the unjam cycle         |
| `unjamReverseDurationSeconds`     | seconds | Reverse phase duration in the unjam cycle         |
| `kS`, `kV`, `kA`                  | volts   | Feedforward gains                                 |
| `kP`, `kI`, `kD`                  | —       | PID velocity controller gains                     |

## Code structure

| File                                           | Purpose                                                            |
| ---------------------------------------------- | ------------------------------------------------------------------ |
| `IndexerSubsystem.java`                        | Concrete subsystem managing roller velocity and ready-to-feed flag |
| `commands/IndexerSubsystemCommandFactory.java` | Factory for feed, hold, reverse, unjam, fire-when-ready, and idle  |
| `commands/FeedCommand.java`                    | Drives the roller at feed RPM                                      |
| `commands/HoldCommand.java`                    | Holds the roller at 0 RPM while a piece is staged                  |
| `commands/ReverseIndexerCommand.java`          | Drives the roller in reverse at a configurable RPM                 |
| `commands/UnjamCommand.java`                   | Alternates forward and reverse to free a jammed piece              |
| `commands/IdleIndexerCommand.java`             | Default command holding idle RPM                                   |
| `config/IndexerSubsystemConfig.java`           | Configuration bundle extending `AbstractVelocitySubsystemConfig`   |
| `shared/config/MotorConfig.java`               | Motor-level config (CAN ID, inversion, current limits) (inherited) |
| `devices/IndexerMotor.java`                    | Real-hardware motor wrapper for SparkMax                           |
| `devices/IndexerSimMotor.java`                 | Simulation motor wrapper                                           |

## Status / TODO

### Done

- Subsystem, commands, config, and device wrappers implemented.
- Wired in `RobotContainer` and added to `subsystems.json`.
- Operator left-bumper binding for feed-and-hold.
- Operator D-pad down binding for unjam.
- Operator D-pad left binding for reverse.
- Fire-when-ready composite command available via factory.

### TODO

- Tune PID and feedforward gains on real hardware.
- Update CAN ID in `subsystems.json` once hardware is assigned.
- Add piece-detection sensor logic for accurate shot counting.
- Add Elastic and AdvantageScope dashboard tabs for tuning.
