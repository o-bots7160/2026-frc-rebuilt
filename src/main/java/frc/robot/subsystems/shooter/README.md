# Shooter subsystem

## Overview

The Shooter subsystem spins up [flywheels](../../GLOSSARY.md#flywheel) to launch
[FUEL](../../GLOSSARY.md#fuel) across the field or into the scoring hub. It
needs steady [RPM](../../GLOSSARY.md#rpm) so volleys stay accurate during
REBUILT matches, with close-range and long-range setpoints to cover different
scoring zones.

## How it works

<!-- TODO: fill in when hardware decisions are made -->

The shooter uses one or more flywheel wheels driven by motors. A
[feedforward](../../GLOSSARY.md#feedforward) term maintains the target velocity,
and a [PID](../../GLOSSARY.md#pid) controller corrects for disturbances when the
[indexer](../indexer/README.md) loads a piece and momentarily slows the wheels.

Planned behaviors:

- **Spin-up to target** — commands set a target RPM and the shooter ramps to it
  using a
  [trapezoidal motion profile](../../GLOSSARY.md#trapezoidal-motion-profile) or
  direct velocity control.
- **Ready-to-fire signal** — a boolean that goes true when the actual RPM is
  within a configurable tolerance of the target and has been stable for a
  minimum time. The [indexer](../indexer/README.md) and autonomous commands wait
  on this signal before feeding.
- **Distance presets** — pre-tuned RPM values for common scoring distances so
  drivers can select them quickly.
- **Spin-down/idle** — when no shot is requested the shooter drops to idle speed
  to save battery, with a quick-spool option for defense-heavy cycles where
  reaction time matters.
- **Reverse** — runs wheels backward for clearing stuck pieces.

## Configuration

<!-- TODO: add config fields and tunables once hardware is selected -->

Expected settings (to be added to `subsystems.json`):

| Setting                                             | Units                        | Purpose                                                 |
| --------------------------------------------------- | ---------------------------- | ------------------------------------------------------- |
| `enabled`                                           | —                            | Master enable flag                                      |
| target RPMs                                         | [RPM](../../GLOSSARY.md#rpm) | Preset velocities for each scoring distance             |
| RPM tolerance                                       | RPM                          | Window around target that counts as "ready"             |
| settle time                                         | seconds                      | How long RPM must stay in tolerance before ready signal |
| [feedforward](../../GLOSSARY.md#feedforward) kS, kV | volts, volts·s/rotation      | Static friction and velocity gains                      |
| [PID](../../GLOSSARY.md#pid) kP, kI, kD             | —                            | Velocity controller gains                               |

## Code structure

<!-- TODO: update when classes are added -->

Planned files:

| File                                  | Purpose                                                                                   |
| ------------------------------------- | ----------------------------------------------------------------------------------------- |
| `ShooterSubsystem.java`               | Subsystem managing flywheel velocity and ready-to-fire state                              |
| `commands/ShooterCommandFactory.java` | Factory for spin-up, idle, preset, and reverse commands                                   |
| `config/ShooterSubsystemConfig.java`  | Configuration and [tunables](../../GLOSSARY.md#tunable)                                   |
| `io/ShooterIO.java`                   | [IO](../../GLOSSARY.md#io-inputoutput) interface for flywheel motors and velocity sensors |

## Status / TODO

### Done

- README and folder structure created.

### TODO

- Choose wheel count, motor types, and sensors.
- Create IO interface and config class.
- Implement velocity control with feedforward + PID.
- Add distance-based RPM presets with tuning support.
- Wire subsystem in `RobotContainer` and add to `subsystems.json`.
