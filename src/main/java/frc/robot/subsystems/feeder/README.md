# Feeder subsystem

## Overview

The Feeder subsystem transports [FUEL](../../GLOSSARY.md#fuel) from the
[intake](../intake/README.md) toward the [indexer](../indexer/README.md). It
uses a belt or roller driven by a single motor to keep pieces moving through the
middle lane of the robot during REBUILT cycles. The feeder also drives the
rollers that were previously part of a separate hopper mechanism, consolidating
transport into a single subsystem.

## How it works

The feeder sits between the intake and the indexer. It extends
`AbstractVelocitySubsystem` for bidirectional velocity control with feedforward
and PID. Positive RPM moves Fuel forward (toward the indexer); negative RPM
reverses the belt (toward the intake).

Behaviors:

- **Idle forward** — the default command keeps the belt spinning at a low
  forward RPM (`idleVelocityRpm`) so Fuel continuously moves toward the indexer
  without operator intervention.
- **Forward transport** — a higher-speed forward mode (`forwardVelocityRpm`) for
  active feeding during scoring cycles.
- **Reverse/clear** — spins the belt backward (`reverseVelocityRpm`) to back
  Fuel out when the operator needs to clear a jam.

## Configuration

Settings are loaded from `subsystems.json` under the `feederSubsystem` key. All
RPM values represent belt (mechanism) speed after gear reduction, not motor
shaft speed.

| Setting                           | Units       | Purpose                                                   |
| --------------------------------- | ----------- | --------------------------------------------------------- |
| `enabled`                         | —           | Master enable flag                                        |
| `verbose`                         | —           | Enables detailed telemetry logging                        |
| `motorConfig.motorCanId`          | —           | CAN ID for the belt motor controller                      |
| `maximumVelocityRpm`              | RPM         | Velocity clamp applied to all targets                     |
| `maximumAccelerationRpmPerSecond` | RPM/s       | Trapezoidal ramp rate (0 = direct PID)                    |
| `velocityToleranceRpm`            | RPM         | Acceptable error for at-target checks                     |
| `settleTimeSeconds`               | seconds     | How long velocity must stay within tolerance before ready |
| `idleVelocityRpm`                 | RPM         | Default forward idle speed                                |
| `forwardVelocityRpm`              | RPM         | Active transport speed toward the indexer                 |
| `reverseVelocityRpm`              | RPM         | Clearing speed toward the intake (stored positive)        |
| `kP`, `kI`, `kD`                  | —           | PID gains for velocity control                            |
| `kS`, `kV`, `kA`                  | volts / ... | Feedforward gains for motor voltage estimation            |

## Code structure

| File                                          | Purpose                                                    |
| --------------------------------------------- | ---------------------------------------------------------- |
| `FeederSubsystem.java`                        | Velocity subsystem with forward/reverse convenience APIs   |
| `commands/IdleFeederCommand.java`             | Default command that holds the belt at the idle RPM        |
| `commands/ReverseFeederCommand.java`          | Command that spins the belt backward to clear Fuel         |
| `commands/FeederSubsystemCommandFactory.java` | Factory for idle, reverse, forward-hold, and SysId cmds    |
| `config/FeederSubsystemConfig.java`           | Configuration and tunables for the feeder                  |
| `shared/config/MotorConfig.java`              | Motor-specific configuration (CAN ID, gearing) (inherited) |
| `devices/FeederMotor.java`                    | SparkMax motor wrapper for real hardware                   |
| `devices/FeederSimMotor.java`                 | Simulation motor wrapper for sim testing                   |

## Status / TODO

### Done

- README and folder structure created.
- Subsystem, config, motor, sim motor, commands, and factory implemented.
- Wired into `SubsystemsConfig`, `RobotContainer`, and all three JSON configs.
- Elastic and AdvantageScope tuning tabs added.

### TODO

- Lock in real motor CAN ID and gear ratio once hardware is selected.
- Run SysId to characterize feedforward gains (kS, kV, kA).
- Tune PID gains for stable velocity tracking.
- Add jam-detection logic (current spike monitoring) and auto-pulse recovery.
