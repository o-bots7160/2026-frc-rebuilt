# Gameplay State subsystem

## Overview

The Gameplay State subsystem tracks the robot's high-level operating mode during
a match. Rather than owning hardware, it maintains a single
[GameplayState](GameplayState.java) enum value that describes what the robot
should be doing right now — collecting Fuel, aiming to shoot, etc.

When the state changes, the companion
[GameplayStateCommandFactory](commands/GameplayStateCommandFactory.java)
composes a parallel command group that coordinates every mechanism subsystem
(shooter, turret, harvester, indexer, feeder, intake) into the correct
configuration. This keeps the operator interface simple: one button press
selects a state, and the factory handles all the details.

## How it works

### States

Each `GameplayState` value represents a coordinated configuration of multiple
subsystems:

| State           | What the robot does                                                               |
| --------------- | --------------------------------------------------------------------------------- |
| `IDLE`          | All mechanisms at rest. Harvester stowed, shooter at idle RPM.                    |
| `HARVEST_READY` | Harvester deployed, intake and feeder running, indexer idle, shooter at idle RPM. |
| `FIRE_READY`    | Shooter spun up, turret tracking the scoring target, indexer feeds when ready.    |
| `AUTO_CYCLE`    | Simultaneous harvest and fire — collects Fuel while the shooter stays spun up.    |
| `EJECT`         | Ball-path mechanisms reverse to clear a jam.                                      |

### Transition sources

Every state change records a human-readable source string so operators and log
reviewers can see **what** triggered the transition:

- `"operator"` — a driver pressed a button bound to a transition command.
- `"fms"` — the FMS match phase changed (e.g., autonomous started).
- `"auto"` — an autonomous routine requested the state.
- `"init"` — the subsystem's initial state at construction.

### Automatic transitions

When `autoTransitionEnabled` is true in the config, the subsystem watches the
FMS match phase each cycle:

1. **Autonomous starts** → transitions to `AUTO_CYCLE` so autonomous routines
   begin collecting and scoring immediately.
2. **Teleop starts** → transitions to `IDLE` so the operator takes control.
3. **Robot disabled** → transitions to `IDLE`.

### Endgame suggestion

The subsystem monitors match time remaining during teleop:

- When match time drops below `endgameThresholdSeconds` (default 30 s), the
  `endgameSuggested` flag is set so dashboards can alert the driver.

### How commands drive the state

- `GameplayStateCommandFactory.createTransitionCommand(state, source)` is the
  central entry point. It records the new state on the subsystem and returns a
  parallel command group for the requested state.
- Each state has a dedicated factory method (e.g., `createFireReadyCommand()`)
  that delegates to individual subsystem command factories.
- The factory does **not** drive motors directly — it composes commands from
  each mechanism's own factory, keeping subsystem ownership intact.

## Configuration

All settings live in `src/main/deploy/subsystems.json` (or the sim/test
variants) under the `gameplayStateSubsystem` key. Values are
[tunable](../../GLOSSARY.md#tunable) at runtime through SmartDashboard when not
connected to the FMS.

### Key tunables

| Setting                   | Type    | Default | Purpose                                               |
| ------------------------- | ------- | ------- | ----------------------------------------------------- |
| `endgameThresholdSeconds` | double  | 30.0    | Seconds remaining in teleop before endgame suggestion |
| `autoTransitionEnabled`   | boolean | true    | Auto-transition on FMS phase changes                  |

## Code structure

| File                                        | Purpose                                                                   |
| ------------------------------------------- | ------------------------------------------------------------------------- |
| `GameplayState.java`                        | Enum of high-level operating modes with display names                     |
| `GameplayStateSubsystem.java`               | Tracks current state, evaluates auto-transitions, publishes telemetry     |
| `commands/GameplayStateCommandFactory.java` | Composes parallel command groups that coordinate all mechanism subsystems |
| `config/GameplayStateSubsystemConfig.java`  | Tunable config for transition behavior and endgame thresholds             |
| `io/GameplayStateIO.java`                   | AdvantageKit IO interface defining telemetry fields                       |

## Status / TODO

### Done

- Full state machine with six operating modes.
- FMS-aware automatic transitions (autonomous, teleop, disabled).
- Endgame timer with configurable threshold and optional auto-climb.
- Parallel command groups coordinating all mechanism subsystems.
- SmartDashboard and AdvantageKit telemetry for current state and transitions.

### TODO

- Add distance-based shooter RPM scaling once vision testing is complete.
- Evaluate whether additional operator feedback (LEDs, rumble) should be tied to
  state transitions.
