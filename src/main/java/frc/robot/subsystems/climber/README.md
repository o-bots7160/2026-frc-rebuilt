# Climber subsystem

## Overview

The Climber subsystem raises the robot onto tower rungs for endgame points in
REBUILT. It extends and retracts arms or hooks for staged climbs: latch a lower
rung, transfer weight, then reach higher. Once the robot is hanging, the climber
holds position under load to prevent backdrive while waiting for the buzzer.

## How it works

<!-- TODO: fill in when hardware decisions are made -->

The climber will use one or more actuators (motors, pneumatics, or a
combination) to drive arms or hooks through a multi-stage climb. Each stage
corresponds to a rung on the tower (1, 2, or 3), and the subsystem should track
which stage the robot has reached.

Planned behaviors:

- **Stage detection** — limit switches or [encoders](../../GLOSSARY.md#encoder)
  placed at each rung position so the driver sees live progress on the
  dashboard.
- **Scripted sequence** — an autonomous climb command runs stages in order with
  clear abort paths if something goes wrong.
- **Manual override** — a backup mode where the operator directly controls
  extension and retraction, useful for recovery when auto-sequences fail.
- **Hold under load** — once latched, the motors brake and current-limit to hold
  the robot's weight without overheating.

## Configuration

<!-- TODO: add config fields and tunables once hardware is selected -->

Expected settings (to be added to `subsystems.json`):

| Setting                                     | Units               | Purpose                               |
| ------------------------------------------- | ------------------- | ------------------------------------- |
| `enabled`                                   | —                   | Master enable flag                    |
| stage positions                             | rotations or inches | Target positions for each rung        |
| current limits                              | amps                | Protect actuators during rung contact |
| [soft limits](../../GLOSSARY.md#soft-limit) | rotations or inches | Prevent over-extension                |

## Code structure

<!-- TODO: update when classes are added -->

Planned files:

| File                                  | Purpose                                                                    |
| ------------------------------------- | -------------------------------------------------------------------------- |
| `ClimberSubsystem.java`               | Subsystem managing actuator state and stage tracking                       |
| `commands/ClimberCommandFactory.java` | Factory for staged-climb and manual-override commands                      |
| `config/ClimberSubsystemConfig.java`  | Configuration and [tunables](../../GLOSSARY.md#tunable)                    |
| `io/ClimberIO.java`                   | [IO](../../GLOSSARY.md#io-inputoutput) interface for actuators and sensors |

## Status / TODO

### Done

- README and folder structure created.

### TODO

- Decide on actuator types, sensors, and rung count.
- Create IO interface and config class.
- Implement staged-climb command sequence with abort logic.
- Add manual override command for operator recovery.
- Wire subsystem in `RobotContainer` and add to `subsystems.json`.
