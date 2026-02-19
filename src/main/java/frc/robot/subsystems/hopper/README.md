# Hopper subsystem

## Overview

The Hopper subsystem holds collected [FUEL](../../GLOSSARY.md#fuel) and moves it
toward the center of the robot. It uses an active floor or belts to pull pieces
from the [intake](../intake/README.md) side toward the
[feeder](../feeder/README.md), spacing them out and preventing bounce-back while
pre-staging for shots.

## How it works

<!-- TODO: fill in when hardware decisions are made -->

The hopper sits between the [intake](../intake/README.md) and the
[feeder](../feeder/README.md). Game pieces entering from the intake land on the
hopper floor, which transports them inward toward the feeder slot.

Planned behaviors:

- **Active floor** — a belt or roller floor keeps pieces moving toward the
  center so the feeder does not starve.
- **Speed coordination** — the hopper speed should match or slightly exceed the
  feeder intake rate to avoid overstuffing the center lane.
- **Piece sensing** — sensors near the feeder exit throttle flow so the
  [indexer](../indexer/README.md) can decide when to fire.
- **Reverse/clear** — a manual reverse mode lets the operator dump pieces back
  toward the intake side to clear jams.
- **Idle by default** — the floor stays stopped until `RobotContainer` triggers
  a staging command.

## Configuration

<!-- TODO: add config fields and tunables once hardware is selected -->

Expected settings (to be added to `subsystems.json`):

| Setting               | Units          | Purpose                                    |
| --------------------- | -------------- | ------------------------------------------ |
| `enabled`             | —              | Master enable flag                         |
| floor speed           | percent or RPM | Forward transport speed                    |
| reverse speed         | percent or RPM | Reverse speed for jam clearing             |
| piece count threshold | count          | Number of pieces that triggers a slow-down |

## Code structure

| File                                          | Purpose                                                              |
| --------------------------------------------- | -------------------------------------------------------------------- |
| `HopperSubsystem.java`                        | Subsystem managing the belt motor for transporting and purging Fuel  |
| `commands/HopperSubsystemCommandFactory.java` | Factory for idle, purge, forward-and-hold, and stop commands         |
| `commands/IdleHopperCommand.java`             | Default command that keeps the belt running forward at idle RPM      |
| `commands/PurgeHopperCommand.java`            | Command that reverses the belt to eject Fuel back through the intake |
| `config/HopperSubsystemConfig.java`           | Configuration and [tunables](../../GLOSSARY.md#tunable)              |
| `config/HopperMotorConfig.java`               | Motor controller configuration (CAN ID, gear ratio, current limit)   |
| `devices/HopperMotor.java`                    | SparkMax motor wrapper for real hardware                             |
| `devices/HopperSimMotor.java`                 | Simulation motor wrapper for testing without hardware                |

## Status / TODO

### Done

- README and folder structure created.
- IO interface, config classes, motor wrappers, and subsystem implemented.
- Idle and purge commands implemented with command factory.
- Subsystem wired in `RobotContainer` with default idle command.
- Configuration added to `subsystems.json`, `subsystems-sim.json`, and
  `subsystems-test.json`.
- Elastic and AdvantageScope tuning layouts added.

### TODO

- Define sensor placement and motor configuration once mechanical packaging is
  set.
- Update CAN ID, gear ratio, and current limit in config when hardware is
  selected.
- Run SysId characterization to determine feedforward gains (kS, kV, kA).
- Tune PID gains for accurate velocity tracking.
- Implement floor coordination with feeder subsystem.
- Add piece-count or sensor-based throttling.
- Add purge button binding in `TriggerBindings`.
