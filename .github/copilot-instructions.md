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
- Subsystems should not manufacture commands. Keep command implementations under
  the relevant `commands` packages; for set-and-seek style mechanisms, use
  `AbstractSetAndSeekCommand` (or a concrete subclass) to drive an
  `AbstractSetAndSeekSubsystem`.
  - Command factories live alongside commands in each subsystem's `commands/`
    folder (no standalone `factories/` folder).
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

- Keep all Java code under `src/main/java/frc/robot` and mirror the current
  layout so teammates can find things quickly:
  - `devices/` holds reusable device wrappers (e.g., controllers).
  - `shared/` contains cross-cutting pieces used by many mechanisms:
    - `bindings/` for trigger/input helpers.
    - `commands/` for abstract command bases.
    - `config/` for shared config types/loaders.
    - `logging/` for AdvantageKit and telemetry helpers.
    - `subsystems/` for abstract subsystem bases.
  - `subsystems/<mechanism>/` houses concrete mechanisms, each with:
    - `commands/` for that subsystem’s commands **and** command factories.
    - `config/` for mechanism-specific settings.
    - `io/` for hardware/sim I/O implementations.
- Keep deployment assets under `src/main/deploy` (e.g., `subsystems.json` and
  mechanism configs like the swerve `controllerproperties.json`, module
  definitions, and trajectories).
- Config classes should surface tunable suppliers for every runtime-adjustable
  value (numbers and booleans) using the AdvantageKit-backed helpers (e.g.,
  `readTunableNumber`, `readTunableBoolean`) so we can tweak on-robot without
  redeploying. Store human-friendly defaults in the fields, then expose
  `get*Supplier()` accessors that read the tunable values.
- Subsystem folders may include a `README.md`; read it for a quick brief on
  behavior, key classes, and configuration before editing or generating code.

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
- When introducing robotics, mechanics, or other domain jargon, add a short
  explanation in Javadoc or a nearby comment so students can learn the term.
- When a subsystem is disabled, add a log message for any public method that
  gets called so operators can see the call was skipped.
- Place comments on their own lines directly **before** the code block they
  describe—avoid end-of-line commentary.
- Remove obsolete or redundant comments during refactors to prevent drift.

## Config naming and units

- All config properties must state their units in the name when applicable (e.g., `maximumSetpointDegrees`, `maximumAngularSpeedDegreesPerSecond`).

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
- Use SmartDashboard/Shuffleboard only for operator-critical values that drivers
  need live; keep everything else AdvantageKit-only to reduce NetworkTables
  noise, especially as we add more subsystems and generated components.
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
