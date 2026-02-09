# Copilot Instructions

These notes describe how to collaborate with the 7160 2026 robot codebase.
Follow them whenever you add code, documentation, or tests.

> **Model note:** This file is consumed by both Claude Opus 4 and GPT 5.x Codex.
> Write every rule as an explicit, concrete directive—do not rely on either
> model "knowing what you mean." When in doubt, include a short positive example
> and, where useful, a negative example. Both models should treat every bullet
> as a hard requirement unless the word "prefer" or "encouraged" appears.

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
- Set-and-seek mechanisms should follow the latest turret pattern:
  - Read the turret guide first:
    [src/main/java/frc/robot/subsystems/turret/README.md](src/main/java/frc/robot/subsystems/turret/README.md).
  - Use
    [src/main/java/frc/robot/shared/subsystems/AbstractSetAndSeekSubsystem.java](src/main/java/frc/robot/shared/subsystems/AbstractSetAndSeekSubsystem.java)
    for profile logic and
    [src/main/java/frc/robot/shared/commands/AbstractSetAndSeekCommand.java](src/main/java/frc/robot/shared/commands/AbstractSetAndSeekCommand.java)
    for command flow.
  - Keep public APIs in degrees, clamp targets to config limits, and log both
    requested and clamped values.
  - Keep command factories in each subsystem’s `commands/` folder (example in
    the turret folder) and avoid putting command builders on subsystems.
- Organize class members by visibility and role: public API at the top, followed
  by protected, package-private, and private helpers. Group overloads together.
- Keep methods cohesive and self-descriptive. When inline comments are warranted
  (see _Javadoc and comment style reference_), keep them short and place them
  above the relevant block.
- Choose semantic variable names that immediately reveal intent (e.g.,
  `driveVelocityMetersPerSecond`, not `vel`). Include units in the name for
  variables, config properties, and constants whenever it prevents ambiguity
  (e.g., `maximumSetpointDegrees`, `maximumAngularSpeedDegreesPerSecond`).
- Abstractions are encouraged when they reduce duplication, but keep them simple
  enough to understand in a quick code review.

## SOLID design principles

Apply the SOLID principles to keep the codebase maintainable as it grows. When
in doubt, favor the simpler design that still satisfies the principle.

- **Single Responsibility (SRP):** Each class should have one reason to change.
  If a class starts accumulating unrelated duties (e.g., a logging utility that
  also owns environment detection), extract the second concern into its own
  class. Example: `Logger` handles telemetry; `RobotEnvironment` handles robot
  mode detection and Driver Station communication.
- **Open/Closed (OCP):** Prefer extending behavior through new subclasses,
  strategy suppliers, or config-driven toggles rather than modifying existing
  working code. The `AbstractSetAndSeekSubsystem` hierarchy is a good example—
  new mechanisms extend the base without altering it.
- **Liskov Substitution (LSP):** Subclasses must honor the contracts of their
  parent. An `AbstractSubsystem` subclass that silently ignores a required
  override breaks LSP. Document any preconditions or postconditions in Javadoc.
- **Interface Segregation (ISP):** When a consumer only needs one capability
  (e.g., a pose supplier), accept `Supplier<Pose2d>` instead of the full
  subsystem. This is already enforced by the _Subsystem decoupling_ rules.
- **Dependency Inversion (DIP):** High-level subsystems should depend on
  abstractions (suppliers, consumers, interfaces), not on concrete
  implementations of other subsystems. Wire concrete dependencies in
  `RobotContainer` and pass them as constructor parameters.

When adding a new class or method, briefly ask: "Does this class have exactly
one job?" and "Am I depending on an abstraction or a concrete class?" These two
checks catch the most common violations.

## Safety guards and error handling

- Every public mutating method on a subsystem must start with an
  `isSubsystemDisabled()` check and return early when disabled. Call
  `logDisabled("methodName")` so operators can see the skipped call in
  telemetry.
  ```java
  public void setTarget(double degrees) {
      if (isSubsystemDisabled()) {
          logDisabled("setTarget");
          return;
      }
      // ... real logic
  }
  ```
- For high-frequency methods like `periodic()`, logging every cycle is too
  noisy. Either return silently or log once using a boolean flag.
- Use `DriverStation.reportError` for **fatal or high-severity** issues
  (constructor failures, missing hardware). Include the stack trace.
- Use `DriverStation.reportWarning` for **recoverable** situations (auto file
  not pre-loaded, missing starting pose). Do not include a stack trace.
- Use the subsystem's `log.verbose` or `log.recordOutput` for subsystem-scoped
  diagnostics.
- When hardware initialization fails in a constructor, set the subsystem to
  disabled (via `config.enabled = false` or equivalent) so the rest of the robot
  keeps running. Do not let a single sensor or motor failure crash the entire
  program.
- Never throw unchecked exceptions from periodic robot code. Catch, report, and
  degrade gracefully.

## Subsystem decoupling

- Subsystems must never hold direct references to other subsystems.
- All cross-subsystem data flows through `Supplier<T>`, `Consumer<T>`, or
  `BiConsumer<T, U>` wired in `RobotContainer`.
- Pass config getters as method references when a `Supplier<Double>` is needed
  (e.g., `config::getMaximumVelocityDegreesPerSecond`).
- Commands that need data from another subsystem should accept suppliers in
  their constructor rather than querying the subsystem directly.
- Good (decoupled):
  ```java
  new AprilTagVisionSubsystem(
          config,
          fieldLayout,
          robotStateSubsystem::addVisionMeasurement,
          driveBaseSubsystem::getOdometryPose);
  ```
- Bad (tightly coupled):
  ```java
  new AprilTagVisionSubsystem(config, fieldLayout, robotStateSubsystem, driveBaseSubsystem);
  ```

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
- Config classes should expose direct getters for every runtime-adjustable value
  (numbers and booleans) using the AdvantageKit-backed helpers (e.g.,
  `readTunableNumber`, `readTunableBoolean`) so we can tweak on-robot without
  redeploying. Store human-friendly defaults in the fields, then expose `get*()`
  accessors that read the tunable values. When a supplier is needed, wrap it at
  the call site (for example, `config::getMaximumVelocityDegreesPerSecond`).
- Prefer the shared helpers in `AbstractConfig` for degree/radian reads:
  `readTunableDegrees` and `readTunableDegreesAsRadians`.
- Subsystem folders may include a `README.md`; read it for a quick brief on
  behavior, key classes, and configuration before editing or generating code.

## FRC and WPILib conventions

- Prefer the WPILib command-based structure: wire devices and subsystems in
  `RobotContainer`, bind inputs in one place, and set default commands so every
  subsystem has an idle behavior.
- Keep control constants in config classes and document their units; reuse them
  in both autonomous and teleop commands to avoid drift.
- Stick to WPILib coordinate frames: field-relative poses use meters and
  radians, chassis speeds are forward (x), left (y), and counter-clockwise
  rotation (omega).
- Schedule one `CommandScheduler.run()` call in `Robot.robotPeriodic()` and
  avoid creating duplicate schedulers or manual loops.
- Use WPILib `Units` helpers (or AdvantageKit math utilities) when converting
  between rotations, radians, meters, and inches; never hard-code magic
  conversion numbers inline.
- For telemetry and logging conventions, see _Logging and telemetry_.
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
  explanation in Javadoc or a nearby comment so students can learn the term. See
  also the domain-jargon guidance in _Additional suggestions_.
- Remove obsolete or redundant comments during refactors to prevent drift.
- For comment placement and formatting rules, see _Javadoc and comment style
  reference_.

## Javadoc and comment style reference

Follow these rules exactly so every file in the codebase looks like it was
written by the same person. When generating or editing code, match these
patterns—do not invent new formatting.

### Class-level Javadoc

- One to three sentences of plain prose. Start with a noun phrase that says what
  the class _is_ or _does_.
- Use a `<p>` block for a second paragraph when you need a usage note or caveat.
  Do **not** use `<h3>`, `<ol>`, `<li>`, `<ul>`, `<strong>`, or `<em>` in any
  Javadoc.
- Add `@param <T>` when the class has type parameters.
- Add `@see` links only when they point to genuinely useful external docs.
- Good example:
  ```java
  /**
   * Factory that creates drive base commands and wires default behaviors.
   */
  ```
- Bad example (too much HTML, too verbose):
  ```java
  /**
   * <h3>DriveBaseSubsystemCommandFactory</h3>
   * <p><strong>This factory</strong> is responsible for...</p>
   * <ol><li>Step one</li><li>Step two</li></ol>
   */
  ```

### Constructor Javadoc

- Summary sentence starts with "Creates…" or "Builds…".
- `@param` entries state purpose and units.
- Good:
  `Creates a factory that produces commands operating on the provided drive base subsystem.`

### Method Javadoc

- Summary sentence starts with an imperative verb: "Builds…", "Computes…",
  "Reads…", "Limits…", "Maps…".
- Use a `<p>` block for caveats or edge-case behavior—keep it to one paragraph.
- `@param` entries include units when the value has them (degrees, meters per
  second, etc.).
- `@return` states what comes back and its units.
- Good:
  ```java
  /**
   * Computes the turret target angle needed to face a field-relative target.
   * <p>
   * The returned angle is clamped to the configured turret limits.
   * </p>
   *
   * @param robotPose                 current robot pose in meters and radians
   * @param targetFieldPositionMeters target position on the field in meters
   * @return turret target angle in degrees
   */
  ```

### Field Javadoc

- One-sentence summary on fields that are not immediately obvious from the name.
- Add a `<p>` block only when extra context helps (domain explanation, wiring
  note).
- Skip Javadoc on trivially obvious private fields.

### Inline comments

- Always on their own line **above** the code they describe, never at the end of
  a line.
- Short imperative phrases, starting with a capital letter. Optionally use a
  label prefix for related groups (e.g., `// Deadband:`, `// Telemetry:`).
- Do **not** use section separator lines (`// ── Section ──`), ASCII-art
  banners, or numbered step comments (`// 1.`, `// 2.`). Let method structure
  and naming convey flow.

### Allowed Javadoc HTML tags

Only the following are permitted:

| Tag                          | Usage                                |
| ---------------------------- | ------------------------------------ |
| `<p>...</p>`                 | Paragraph break inside Javadoc       |
| `{@link ClassName}`          | Cross-reference to a class or method |
| `{@link ClassName#method()}` | Cross-reference with method anchor   |
| `{@code literal}`            | Inline code literal                  |

Everything else (`<h3>`, `<ol>`, `<li>`, `<ul>`, `<strong>`, `<em>`, `<table>`,
`<pre>`) is **not allowed** in Javadoc.

## Import ordering

- Group imports in this order with a blank line between each group:
  1. `import static` (if any).
  2. `java.*` / `javax.*` (standard library).
  3. Third-party libraries (`com.*`, `org.*`, vendor libs).
  4. `edu.wpi.first.*` (WPILib).
  5. `frc.robot.*` (project code).
- Do not use wildcard imports (`import java.util.*`). Import each class
  individually so dependencies are explicit and diffs are clean.

## RobotContainer wiring order

- Follow this sequence in the `RobotContainer` constructor so every dependency
  is available before it is referenced:
  1. **Configuration loading** — config objects, field layout suppliers.
  2. **Subsystems** — instantiate in dependency order (e.g.,
     `RobotStateSubsystem` before `DriveBaseSubsystem` when drivebase publishes
     odometry into robot state).
  3. **Cross-subsystem wiring** — connect consumers and suppliers between
     subsystems.
  4. **Command factories** — create one factory per subsystem.
  5. **Default commands** — set idle behaviors.
  6. **Input bindings** — map controller buttons to commands.
- When adding a new subsystem, insert it in the correct dependency position. Do
  not add it at the bottom of the list if other subsystems depend on it.

## Disabled-subsystem lifecycle

Follow this pattern when adding a new subsystem that can be toggled off:

1. Add an `enabled` field to the subsystem entry in `subsystems.json`,
   `subsystems-sim.json`, and `subsystems-test.json`.
2. The corresponding config class inherits `public boolean enabled = true` from
   `AbstractConfig`.
3. `AbstractSubsystem` copies the flag in its constructor, exposing
   `isSubsystemDisabled()` and `logDisabled(String)`.
4. Guard public methods as described in _Safety guards and error handling_.
5. In the subsystem constructor, bail out early when disabled:
   ```java
   if (isSubsystemDisabled()) {
       log.verbose("MySubsystem disabled; skipping hardware init.");
       this.io = inputs -> {};
       return;
   }
   ```
6. For motor-backed subsystems, use `DisabledMotor` (or an equivalent no-op IO
   implementation) when the motor is absent so callers never need null checks.
7. `RobotContainer` still constructs all subsystems even when disabled. They
   become inert, keeping wiring simple and avoiding null references.

## Autonomous and PathPlanner conventions

- Auto files live under `src/main/deploy/pathplanner/autos/` and follow the
  naming pattern `Start Position N - Action.auto` (e.g.,
  `Start Position 1 - Shoot and Collect.auto`).
- Path files live under `src/main/deploy/pathplanner/paths/` and follow
  `Start Position N - Segment Name.path`.
- The `resolveAutoName` switch in `PathPlannerCommandFactory` must stay in sync
  with the deployed `.auto` files. When adding a new auto, update both.
- Pre-load all autos at construction time (in the factory constructor) to avoid
  file-parsing delays when autonomous is enabled during a match.
- On-the-fly alignment paths should set `preventFlipping = true` when
  coordinates have already been adjusted for the current alliance.

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
- In subsystems, use the shared `log` from `AbstractSubsystem` for telemetry and
  input logging (for example, `log.recordOutput` and `log.processInputs`) rather
  than calling the static AdvantageKit logger directly.
- Use SmartDashboard/Shuffleboard only for operator-critical values that drivers
  need live; keep everything else AdvantageKit-only to reduce NetworkTables
  noise, especially as we add more subsystems and generated components.
- When adding detailed telemetry, wrap it in a verbose or debug flag so it can
  be disabled quickly for events; document how to toggle the flag in code or
  config.
- Avoid logging inside high-frequency loops unless the data is throttled or
  summarized—favor per-cycle aggregates or periodic snapshots where possible.

## Additional suggestions

- SmartDashboard keys should be prefixed with the subsystem name (e.g.,
  `DriveBaseSubsystem/headingKp`), not the config class name (`*Config`). Apply
  this pattern for all subsystems and configs going forward.
- When adding device wrappers or helpers, provide a brief usage example in the
  class-level Javadoc.
- Keep TODOs actionable and include an owner or link so they do not linger
  without context.
- Never introduce inline magic numbers. Extract every numeric literal into a
  named constant or config field with units in the name.
- When generating code for students, favor clarity over cleverness. A slightly
  longer solution that reads like prose is better than a compact one that
  requires advanced Java knowledge to parse.
- If a method grows beyond roughly 30–40 lines, consider extracting helpers.
  Short methods with descriptive names are easier for students to step through
  in a debugger.
- When a concept has a real-world robotics term (PID, feedforward, trapezoidal
  profile, odometry, holonomic), define it briefly in the Javadoc the first time
  it appears in a class so students learn the vocabulary as they read the code.
- Commit messages should use imperative mood ("Add turret tracking command", not
  "Added" or "Adding"). Keep the summary under 72 characters and add a body
  paragraph for non-trivial changes.
