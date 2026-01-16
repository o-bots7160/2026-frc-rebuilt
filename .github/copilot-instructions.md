# Copilot Instructions

These notes describe how to collaborate with the 7160 2026 robot codebase.
Follow them whenever you add code, documentation, or tests.

## Writing voice and documentation expectations

- Keep explanations at a clear high-school reading level—short sentences,
  concrete nouns, and actionable verbs.
- Every new public or protected method **must** include Javadoc with:
  - A one-sentence summary explaining what the method accomplishes and when to
    call it.
  - Usage guidance that highlights important behaviors, assumptions, or side
    effects.
  - `@param` entries that explain what each argument represents, units (degrees,
    radians, rotations, meters, etc.), and why it matters.
  - `@return` and `@throws` tags whenever applicable.
- Prefer prose documentation (README, package-info, or subsystem docs) before
  adding inline comments.

## Code structure and conventions

- Follow standard Java conventions: `UpperCamelCase` classes, `lowerCamelCase`
  methods/fields, constants in `UPPER_SNAKE_CASE`.
- Naming standards:
  - Commands must end with `Command` (e.g., `MoveFieldManualCommand`).
  - Subsystems must end with `Subsystem` (e.g., `DriveBaseSubsystem`).
  - Factories/helpers should use descriptive nouns that reveal purpose.
- Subsystems should not manufacture commands. Keep command implementations under the relevant `commands` packages; for set-and-seek style mechanisms, use
  `AbstractSetAndSeekCommand` (or a concrete subclass) to drive an `AbstractSetAndSeekSubsystem`.
- Organize class members by visibility and role: public API at the top, followed
  by protected, package-private, and private helpers. Group overloads together.
- Keep methods cohesive and self-descriptive. If logic is hard to infer from
  names alone, add a short comment **above** the relevant block.
- Choose semantic variable names that immediately reveal intent (e.g.,
  `driveVelocityMetersPerSecond`, not `vel`). Include units in the name when it
  prevents ambiguity.
- Abstractions are encouraged when they reduce duplication, but keep them simple
  enough to understand in a quick code review.

## Project layout

- Place all robot code under `src/main/java/frc/robot`, mirroring the existing
  structure:
  - `config` for configuration objects or loaders.
  - `subsystems` for subsystem classes and their helpers.
  - `helpers` for reusable utilities (logging, math helpers, etc.).
- When creating new files, mirror the folder naming pattern so other
  contributors can find code quickly.
- Keep deployment assets (JSON configs, trajectories) under `src/main/deploy`.

## FRC and WPILib conventions

- Prefer the WPILib command-based structure: wire devices and subsystems in
  `RobotContainer`, bind inputs in one place, and set default commands so every
  subsystem has an idle behavior.
- Keep control constants in one location (config classes or a `Constants`
  helper) and document their units; reuse them in both autonomous and teleop
  commands to avoid drift.
- Stick to WPILib coordinate frames: field-relative poses use meters and
  radians, chassis speeds are forward (x), left (y), and counter-clockwise
  rotation (omega).
- Schedule one `CommandScheduler.run()` call in `Robot.robotPeriodic()` and
  avoid creating duplicate schedulers or manual loops.
- Use WPILib `Units` helpers (or AdvantageKit math utilities) when converting
  between rotations, radians, meters, and inches; never hard-code magic
  conversion numbers inline.
- Default to AdvantageKit logging for telemetry and prefer
  `SmartDashboard`/`Shuffleboard` only for quick debug values.
- When adding new hardware, supply vendor IDs and CAN IDs in config classes,
  ensure `SubsystemsConfig` stays in sync with `subsystems.json`, and document
  any Phoenix/REV firmware needs.
- Sim-friendly code is encouraged: gate hardware-only calls behind WPILib
  simulation checks and provide reasonable fallbacks so `./gradlew simulateJava`
  stays usable.

## Comments and readability

- Favor expressive code over comments. Only add comments when the intent cannot
  be captured by naming alone.
- Place comments on their own lines directly **before** the code block they
  describe—avoid end-of-line commentary.
- Remove obsolete or redundant comments during refactors to prevent drift.

## Testing and validation

- When logic changes could affect runtime behavior, add or update
  tests/simulations where possible and run `./gradlew build` locally before
  committing.
- After editing runnable code, prefer running `./gradlew build` to catch
  regressions; keep builds frequent as we add more generated subsystems.
- Mention any manual driver-station checks or hardware requirements in PR
  descriptions so reviewers know how to verify changes.

## Logging and telemetry

- Default to AdvantageKit (`org.littletonrobotics.junction.Logger`,
  `LoggedDashboardValue`) for telemetry; record structured values with
  `Logger.recordOutput` instead of SmartDashboard calls.
- Use SmartDashboard/Shuffleboard only for operator-critical values that
  drivers need live; keep everything else AdvantageKit-only to reduce
  NetworkTables noise, especially as we add more subsystems and generated
  components.
- When adding detailed telemetry, wrap it in a verbose or debug flag so it can
  be disabled quickly for events; document how to toggle the flag in code or
  config.
- Avoid logging inside high-frequency loops unless the data is throttled or
  summarized—favor per-cycle aggregates or periodic snapshots where possible.

## Additional suggestions

- Surface important configuration constants via descriptive names and document
  their units in both code and config files.
- Whenever you introduce a new tuning value (feedforward gains, current limits,
  mechanical offsets, etc.), extract it into the relevant config class or
  `subsystems.json` entry so that future adjustments happen in one location.
- Centralize repeated tuning parameters in config classes so that changes
  propagate consistently across autonomous and teleop code.
- SmartDashboard keys should be prefixed with the subsystem name (e.g.,
  `DriveBaseSubsystem/headingKp`), not the config class name (`*Config`). Apply
  this pattern for all subsystems and configs going forward.
- When adding device wrappers or helpers, provide a brief usage example in the
  class-level Javadoc.
- Keep TODOs actionable and include an owner or link so they do not linger
  without context.
