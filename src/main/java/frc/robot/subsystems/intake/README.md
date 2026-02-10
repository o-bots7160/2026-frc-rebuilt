# Intake subsystem

## Overview

The Intake subsystem grabs loose [FUEL](../../GLOSSARY.md#fuel) from the carpet
and hands it to the [hopper](../hopper/README.md). It deploys rollers that pull
game pieces from the field and pass them straight into the hopper entrance. The
intake has no storage of its own — it is a pass-through that runs fast in teleop
for cycle speed and retracts when stray pieces are not wanted.

## How it works

<!-- TODO: fill in when hardware decisions are made -->

The intake is the first mechanism a game piece touches after leaving the field.
It is typically mounted at the front of the robot with a deployable arm or
fixed-position rollers.

Planned behaviors:

- **Deploy and spin** — a single command extends the intake and spins the
  rollers inward to grab pieces off the carpet.
- **Retract and stop** — pulling the intake in protects it during travel and
  prevents accidental collection in protected zones.
- **Outtake/reverse** — reverses the rollers to eject a piece, useful for
  clearing jams or discarding unwanted game pieces.
- **Jam detection** — monitors motor current to detect stuck pieces and
  auto-pulses the rollers to free them.
- **Idle by default** — rollers stay stopped and the intake stays retracted
  until the operator presses the intake button in `RobotContainer`.
- **Sim support** — timing mirrors real hardware so
  [hopper](../hopper/README.md)/[feeder](../feeder/README.md) logic can still be
  tested in simulation.

## Configuration

<!-- TODO: add config fields and tunables once hardware is selected -->

Expected settings (to be added to `subsystems.json`):

| Setting                  | Units                                   | Purpose                                   |
| ------------------------ | --------------------------------------- | ----------------------------------------- |
| `enabled`                | —                                       | Master enable flag                        |
| roller speed             | percent or [RPM](../../GLOSSARY.md#rpm) | Inward roller output                      |
| reverse speed            | percent or RPM                          | Outtake roller output                     |
| jam current threshold    | amps                                    | Current level that triggers jam detection |
| deploy/retract positions | rotations or degrees                    | Arm positions (if applicable)             |

## Code structure

<!-- TODO: update when classes are added -->

Planned files:

| File                                 | Purpose                                                                                 |
| ------------------------------------ | --------------------------------------------------------------------------------------- |
| `IntakeSubsystem.java`               | Subsystem managing roller and deploy/retract control                                    |
| `commands/IntakeCommandFactory.java` | Factory for intake, outtake, and jam-clear commands                                     |
| `config/IntakeSubsystemConfig.java`  | Configuration and [tunables](../../GLOSSARY.md#tunable)                                 |
| `io/IntakeIO.java`                   | [IO](../../GLOSSARY.md#io-inputoutput) interface for rollers, deploy motor, and sensors |

## Status / TODO

### Done

- README and folder structure created.

### TODO

- Flesh out command list and sensor strategy once hardware layout and wiring are
  finalized.
- Create IO interface and config class.
- Implement deploy/retract with position tracking.
- Add jam-detection with auto-pulse recovery.
- Wire subsystem in `RobotContainer` and add to `subsystems.json`.
