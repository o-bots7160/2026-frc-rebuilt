# Drivebase subsystem

This folder holds everything that moves the robot on the field. It wraps the
swerve hardware, exposes a simple API for commands, and keeps logging wired up.

## What lives here

- `DriveBaseSubsystem` – the main class that owns odometry, field-relative
  driving, and pose targeting using YAGSL.
- `commands/MoveFieldManualCommand` – turns driver stick inputs into
  field-relative chassis speeds.
- `config/DriveBaseSubsystemConfig` – loads tuning values (max speeds,
  tolerances) and links to the swerve JSON files in `src/main/deploy/swerve/`.
- `factories/DriveBaseSubsystemCommandFactory` – builds drive commands and sets
  the default manual drive command.
- `io/DriveBaseIO` and `io/DriveBaseIOYagsl` – interface and YAGSL-backed
  implementation that feed sensor data to AdvantageKit.

## How it behaves

- Uses field-centric control so forward on the stick always means field +X,
  regardless of robot facing.
- Publishes odometry, module states, and targets to AdvantageKit for replay and
  tuning.
- Holds optional target poses for autonomous-style aiming and uses PID
  controllers to drive toward them.

## Configuration notes

- Hardware geometry and module tuning live in the deploy files under
  `src/main/deploy/swerve/` (controller properties, module definitions, PIDF
  gains).
- `subsystems.json` controls whether the drivebase is enabled for a given build;
  when disabled, the subsystem skips hardware init for sim-only runs.

## When to edit

- Add or tweak commands when driver controls or auto behaviors change.
- Update config values when you retune speeds, deadbands, or tolerances.
- Adjust the deploy JSON files if hardware IDs, geometry, or module settings
  change.

## For Copilot and reviewers

- Treat this README as the quick brief before changing drive code; keep it
  updated when behaviors or key classes change.
- Preserve field-centric control and AdvantageKit logging; new commands should
  reuse `DriveBaseSubsystem` helpers and log important targets and states.
- Keep command names ending in `Command` and factory helpers in
  `factories/DriveBaseSubsystemCommandFactory`.
