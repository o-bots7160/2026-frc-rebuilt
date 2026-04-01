# Intake Subsystem

## Overview

The intake subsystem spins rollers to pull [Fuel](../../GLOSSARY.md#fuel) from
the carpet and push it into the [feeder](../feeder/README.md). It has no storage
of its own — it is a pass-through that runs fast in teleop for cycle speed and
stops when pieces are not wanted.

The arm that deploys and retracts the rollers is managed by the separate
[harvester](../harvester/README.md) subsystem. The two subsystems share no
direct references and are composed through trigger bindings in `RobotContainer`.

## How it works

The intake is the first mechanism a game piece touches after leaving the field.
Rollers spin forward (positive RPM) to grab Fuel off the carpet and push it into
the feeder, or reverse (negative RPM) to eject unwanted pieces back onto the
field. When no command is active, the default idle command holds the rollers at
0 RPM.

## Key classes

| File                                          | Purpose                                           |
| --------------------------------------------- | ------------------------------------------------- |
| `IntakeSubsystem.java`                        | Velocity subsystem with forward/reverse helpers   |
| `config/IntakeSubsystemConfig.java`           | Forward/reverse RPM, PID, tolerance, and tunables |
| `shared/config/MotorConfig.java`              | CAN ID, gear ratio, current limit (inherited)     |
| `devices/IntakeMotor.java`                    | SparkMax velocity motor wrapper                   |
| `devices/IntakeSimMotor.java`                 | Simulation velocity motor for testing             |
| `commands/IdleIntakeCommand.java`             | Default command — holds idle RPM (0)              |
| `commands/EjectIntakeCommand.java`            | Reverses rollers at configured eject RPM          |
| `commands/IntakeSubsystemCommandFactory.java` | Factory for idle, eject, intake, and continuous   |

## Configuration

All velocities are stored in RPM in `subsystems.json` (and sim/test variants).
The subsystem config provides tunable getters so speeds can be adjusted on the
fly from Elastic without redeploying.

| Setting              | Units | Purpose                            |
| -------------------- | ----- | ---------------------------------- |
| `enabled`            | —     | Master enable flag                 |
| `forwardVelocityRpm` | RPM   | Inward roller speed for collection |
| `reverseVelocityRpm` | RPM   | Reverse roller speed for ejection  |
| `idleVelocityRpm`    | RPM   | Speed when idle (typically 0)      |
| `maximumVelocityRpm` | RPM   | Motor velocity ceiling             |
