# Feeder subsystem

## Overview

The Feeder subsystem centers [FUEL](../../GLOSSARY.md#fuel) inside the hopper so
the [indexer](../indexer/README.md) always receives a clean handoff. It pulls
pieces from the left or right side into the middle lane and hands them forward,
keeping the conveyor balanced during REBUILT cycles.

## How it works

<!-- TODO: fill in when hardware decisions are made -->

The feeder sits between the [hopper](../hopper/README.md) and the
[indexer](../indexer/README.md). It uses one or more belts or rollers to move
game pieces toward the center of the robot.

Planned behaviors:

- **Center-feed** — pull pieces from whichever side has them into the middle
  lane, coordinating speed with the hopper to avoid double-feeding.
- **Jam detection** — monitor motor current spikes or proximity sensors to
  detect side jams, then switch to a pulse/alternate pattern to clear them.
- **Reverse/clear** — a quick reverse mode to back pieces out when the operator
  needs to clear a jam manually.
- **Idle by default** — the feeder stays stopped until intake is active and
  `RobotContainer` triggers a centering command.

## Configuration

<!-- TODO: add config fields and tunables once hardware is selected -->

Expected settings (to be added to `subsystems.json`):

| Setting               | Units          | Purpose                                   |
| --------------------- | -------------- | ----------------------------------------- |
| `enabled`             | —              | Master enable flag                        |
| belt speeds           | percent or RPM | Forward and reverse motor outputs         |
| jam current threshold | amps           | Current level that triggers jam detection |

## Code structure

<!-- TODO: update when classes are added -->

Planned files:

| File                                 | Purpose                                                                      |
| ------------------------------------ | ---------------------------------------------------------------------------- |
| `FeederSubsystem.java`               | Subsystem managing left/right belt control and jam detection                 |
| `commands/FeederCommandFactory.java` | Factory for center-feed, reverse, and jam-clear commands                     |
| `config/FeederSubsystemConfig.java`  | Configuration and [tunables](../../GLOSSARY.md#tunable)                      |
| `io/FeederIO.java`                   | [IO](../../GLOSSARY.md#io-inputoutput) interface for belt motors and sensors |

## Status / TODO

### Done

- README and folder structure created.

### TODO

- Lock in motor layout and sensing approach.
- Create IO interface and config class.
- Implement centering command with hopper speed coordination.
- Add jam-detection logic and auto-pulse recovery.
- Wire subsystem in `RobotContainer` and add to `subsystems.json`.
