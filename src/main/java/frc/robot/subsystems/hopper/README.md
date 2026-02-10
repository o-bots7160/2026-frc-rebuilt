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

<!-- TODO: update when classes are added -->

Planned files:

| File                                 | Purpose                                                                       |
| ------------------------------------ | ----------------------------------------------------------------------------- |
| `HopperSubsystem.java`               | Subsystem managing the active floor and piece sensing                         |
| `commands/HopperCommandFactory.java` | Factory for stage-fuel, reverse, and idle commands                            |
| `config/HopperSubsystemConfig.java`  | Configuration and [tunables](../../GLOSSARY.md#tunable)                       |
| `io/HopperIO.java`                   | [IO](../../GLOSSARY.md#io-inputoutput) interface for floor motors and sensors |

## Status / TODO

### Done

- README and folder structure created.

### TODO

- Define sensor placement and motor configuration once mechanical packaging is
  set.
- Create IO interface and config class.
- Implement floor coordination with feeder subsystem.
- Add piece-count or sensor-based throttling.
- Wire subsystem in `RobotContainer` and add to `subsystems.json`.
