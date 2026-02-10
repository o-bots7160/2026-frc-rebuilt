# Indexer subsystem

## Overview

The Indexer subsystem controls when [FUEL](../../GLOSSARY.md#fuel) leaves the
robot. It acts as the gate between storage
([hopper](../hopper/README.md)/[feeder](../feeder/README.md)) and the
[shooter](../shooter/README.md), metering one piece at a time so shots can be
timed during REBUILT scoring windows. The indexer can hold a preloaded piece for
autonomous or volley sequences.

## How it works

<!-- TODO: fill in when hardware decisions are made -->

The indexer sits directly upstream of the [shooter](../shooter/README.md). It
receives centered pieces from the [feeder](../feeder/README.md) and releases
them into the shooter flywheels on demand.

Planned behaviors:

- **Fire/hold gate** — the indexer does not spin unless the shooter
  [RPM](../../GLOSSARY.md#rpm) is within tolerance and the
  [turret](../turret/README.md) aim is on target.
- **Shot-ready signal** — a boolean flag for autonomous commands to wait on
  before advancing to the next step.
- **Single-step and burst** — commands support feeding one piece at a time or
  running continuously for rapid volleys.
- **Reverse/clear** — a reverse mode backs pieces out when the shooter is
  disabled or a jam is detected.
- **Piece detection** — sensors at the gate detect when a piece is staged and
  when it has been launched, enabling accurate shot counting.

## Configuration

<!-- TODO: add config fields and tunables once hardware is selected -->

Expected settings (to be added to `subsystems.json`):

| Setting                | Units                                   | Purpose                                                     |
| ---------------------- | --------------------------------------- | ----------------------------------------------------------- |
| `enabled`              | —                                       | Master enable flag                                          |
| feed speed             | percent or [RPM](../../GLOSSARY.md#rpm) | Motor output for feeding into shooter                       |
| reverse speed          | percent or RPM                          | Motor output for clearing                                   |
| shot detection timeout | seconds                                 | How long after a feed pulse to wait before declaring a miss |

## Code structure

<!-- TODO: update when classes are added -->

Planned files:

| File                                  | Purpose                                                                           |
| ------------------------------------- | --------------------------------------------------------------------------------- |
| `IndexerSubsystem.java`               | Subsystem managing the gate motor and piece sensing                               |
| `commands/IndexerCommandFactory.java` | Factory for fire, hold, burst, and reverse commands                               |
| `config/IndexerSubsystemConfig.java`  | Configuration and [tunables](../../GLOSSARY.md#tunable)                           |
| `io/IndexerIO.java`                   | [IO](../../GLOSSARY.md#io-inputoutput) interface for gate motor and piece sensors |

## Status / TODO

### Done

- README and folder structure created.

### TODO

- Add hardware map, sensors, and command factory once the physical gate design
  is finalized.
- Implement ready-to-fire signal for autonomous gating.
- Guard against double-commanding the gate when the shooter is not ready.
- Wire subsystem in `RobotContainer` and add to `subsystems.json`.
