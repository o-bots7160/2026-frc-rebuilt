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
- When creating a new subsystem, command, config, or device wrapper, compare it
  against existing siblings in the same abstract hierarchy (e.g., other velocity
  subsystems, other motor wrappers). If two or more concrete classes share
  identical methods, extract the shared logic into the appropriate abstract base
  class to follow DRY. Check subsystems, configs, motors, sim motors, commands,
  and command factories — duplication can appear at any layer.

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

## Formatting generated and edited files

- After creating or editing a Java file, **always** run the VS Code formatter on
  the file before moving on. Use the `editor.action.formatDocument` command (or
  the equivalent agent tool) so the result matches the workspace's formatter
  settings (indentation, brace placement, line length, import order, etc.).
- When multiple files are created or edited in a single task, format each file
  individually after its content is finalized—do not skip formatting because
  "it's just generated code."
- This applies to all file types the workspace formatter supports (Java, JSON,
  Markdown, etc.), not only Java.
- If the workspace has a `.editorconfig`, Checkstyle config, or other formatter
  profile, the VS Code formatter will respect it automatically; no extra flags
  are needed.

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

## Architecture diagram (`7160-frc-rebuilt.drawio.svg`)

The root-level `7160-frc-rebuilt.drawio.svg` is the single source-of-truth
architecture diagram, managed via the `drawio-mcp` MCP server (configured in
`.vscode/mcp.json`). Always read the diagram with `get_diagram_info` before
making changes so you understand current node/edge IDs and positions.

### MCP server reference

- The diagram MCP server is defined in `.vscode/mcp.json` as
  `drawio-diagrams`, using `npx github:brandonmartinez/drawio-mcp`.
- Because it pulls from a GitHub repo (not an npm registry package), clearing
  the npx cache requires deleting the matching directories under
  `~/.npm/_npx/` and then restarting the MCP server from VS Code.
- Available tools: `new_diagram`, `add_nodes`, `edit_nodes`, `link_nodes`,
  `remove_nodes`, `get_diagram_info`.
- `link_nodes` supports `edgeStyle` (`orthogonal`, `segment`, `elbow`,
  `entity-relation`, `straight`) and `waypoints` (array of `{x, y}` objects)
  for multi-point routing.

### Diagram layout structure

The diagram is arranged in horizontal bands, top to bottom:

1. **Legend row** (y ≈ 0) — color-coded key boxes at the very top.
2. **Robot / RobotContainer bar** (y = 50–120) — full-width blue bars.
3. **Configuration** (y = 130) — yellow config box, right-aligned.
4. **Subsystem row** (y = 200) — all subsystem nodes in a horizontal line,
   grouped by function:
   - DRIVE & POSITIONING (x = 0–880): DriveBase, Turret, Climber.
   - SENSING & VISION (x = 920–1800): RobotState, AprilTagVision,
     DriverCamera.
   - BALL PATH (x = 1820–2360): Shooter, Indexer, Feeder.
5. **Command factory row** (y = 440–660) — purple factory nodes below their
   respective subsystems.
6. **Command row** (y = 600–1025) — red command nodes below their factories.
7. **Controller / bindings row** (y = 1040–1180) — yellow driver/operator
   controllers and TriggerBindings bar.

### Node conventions

- Each node type has a consistent color and style. Do not invent new colors:

  | Category         | Fill        | Stroke      | Font style |
  | ---------------- | ----------- | ----------- | ---------- |
  | Subsystem        | `#d5e8d4`   | `#82b366`   | bold       |
  | Command Factory  | `#e1d5e7`   | `#9673a6`   | bold       |
  | Command          | `#f8cecc`   | `#b85450`   | bold       |
  | Config / Binding | `#fff2cc`   | `#d6b656`   | bold       |
  | Infrastructure   | `#dae8fc`   | `#6c8ebf`   | bold       |
  | Data Flow legend | `#dae8fc`   | `#0066cc`   | bold+italic|

- Node IDs use kebab-case with a short descriptive slug: `db-sub`,
  `turret-factory`, `cmd-shooter-idle`, `trigger`.
- Default commands are prefixed with a star emoji: `⭐`.
- Font size is `11` for most nodes, `12–13` for the Robot/RobotContainer bars.
- Use `RoundedRectangle` with appropriate `arcSize` for all nodes (subsystems
  use `arcSize=16` or `arcSize=32`; factories/commands `arcSize=12` or `24`).

### Edge conventions and lane routing

Each edge belongs to one of four semantic categories. Assign the correct
color, style, and routing lane so edges never stack on top of each other.

| Category       | Color       | Style    | Label    | Routing lane   |
| -------------- | ----------- | -------- | -------- | -------------- |
| Data flow      | `#0066CC`   | solid 2px| varies   | Above subsystem row (y = 140–165) using waypoints |
| Owns ref       | `#9673a6`   | solid 1px| "owns ref" | **Left lane** — waypoints ~15–20px outside the left edge of the column |
| Creates        | `#b85450`   | dashed 1px| "creates" | **Center** — direct orthogonal, or right-side waypoints when bypassing intermediate nodes |
| Requires       | `#82b366`   | solid 1px| "requires" | **Right lane** — waypoints ~5–20px outside the right edge of the column |
| Trigger → factory | `#d6b656` | solid 1px| "uses" | Horizontal bus along y = 1020–1040 with waypoints, then vertical to target |

- **Lane separation is critical.** When a subsystem column has all three
  vertical edge types (owns ref, creates, requires), they must run in
  separate vertical lanes so labels do not overlap. Use `segment` edge style
  with explicit waypoints for left/right lanes; use `orthogonal` for center
  connections that go straight down.
- When multiple edges share the same lane (e.g., two "requires" edges to the
  same subsystem), offset their waypoint x-coordinates by 5px each to prevent
  overlap.
- Edge IDs follow the pattern `source-2-target` (e.g.,
  `turret-factory-2-turret-sub`, `cmd-track-2-turret-sub`).
- Font size on all edge labels is `10`.
- Data flow edges (blue) are `strokeWidth=2`; all other edges are default
  width.
- Data flow edges between non-adjacent subsystems use waypoints routed above
  the subsystem row (y = 140–165) to avoid crossing through subsystem nodes.
  Each horizontal data flow line uses a distinct y-value so parallel blue
  lines do not overlap.

### Keeping the diagram in sync

- When adding a new subsystem, add a subsystem node (green), a command
  factory node (purple), and any command nodes (red) in the correct x-region.
  Wire all three edge types (owns ref, creates, requires) using the lane
  conventions above.
- When adding a new command to an existing subsystem, add the command node
  below the existing commands, create a dashed "creates" edge from the
  factory, and a solid "requires" edge back to the subsystem using the right
  lane.
- When adding cross-subsystem data flow, use blue edges with waypoints routed
  above the subsystem row.
- When adding a new trigger binding, add a "uses" edge from `trigger` to the
  relevant factory using waypoints along the horizontal bus (y = 1020–1040).
- Update the driver/operator controller node labels when button mappings
  change.
- Always use `get_diagram_info` first to confirm current node positions and
  IDs before editing. Node positions may have shifted from manual edits in
  the draw.io VS Code extension.

## Elastic Dashboard layout (`elastic-layout.json`)

The root-level `elastic-layout.json` defines the Elastic Dashboard layout,
managed via the `elastic-dashboard` MCP server (configured in
`.vscode/mcp.json`). **Never edit `elastic-layout.json` by hand.** Always use
the MCP tools listed below.

### MCP server reference

- The server is defined in `.vscode/mcp.json` as `elastic-dashboard`, using
  `npx -y @o-bots7160/elastic-dashboard-mcp`.
- Available tools: `create_layout`, `get_layout`, `get_tab`, `add_tab`,
  `remove_tab`, `rename_tab`, `reorder_tabs`, `add_widget`,
  `add_widgets_batch`, `remove_widget`, `move_widget`, `resize_widget`,
  `update_widget_properties`, `auto_layout`, `list_widget_types`,
  `suggest_widget`, `convert_color`, `get_config`, `set_config`,
  `validate_layout`.

### Workflow

1. Use `get_layout` to read the current tab/widget structure before making any
   changes.
2. Use `add_tab` to create a new tab, then `add_widget` or
   `add_widgets_batch` to populate it. Positions are auto-calculated when
   omitted.
3. After changes, run `validate_layout` to confirm the file is well-formed.
4. Use `list_widget_types` (optionally filtered by `single-topic`,
   `multi-topic`, or `layout`) to discover valid widget types and their
   default sizes.
5. Use `suggest_widget` with a NetworkTables data type to choose the best
   widget for a topic.
6. Colors in Elastic Dashboard are decimal ARGB integers. Use
   `convert_color` to translate between hex (`#FF4CAF50`) and the integer
   format stored in the JSON.

### Conventions

- Grid size is `128` px. Widget positions and sizes snap to the grid.
- Widget topics use full SmartDashboard paths (e.g.,
  `/SmartDashboard/GameplayStateSubsystem/CurrentState`).
- Tab names should be concise and descriptive (e.g., `Competition`,
  `State & Pose`, `DriveBase Tuning`).

## AdvantageScope layout (`advantagescope-layout.json`)

The root-level `advantagescope-layout.json` defines the AdvantageScope layout,
managed via the `advantagescope` MCP server (configured in `.vscode/mcp.json`).
**Never edit `advantagescope-layout.json` by hand.** Always use the MCP tools
listed below.

### MCP server reference

- The server is defined in `.vscode/mcp.json` as `advantagescope`, using
  `npx -y @o-bots7160/advantagescope-mcp`.
- Available tools: `create_layout`, `get_layout`, `get_tab`,
  `get_tab_type_schema`, `list_tab_types`, `add_hub`, `remove_hub`,
  `add_tab`, `remove_tab`, `update_tab`, `move_tab`, `reorder_tabs`,
  `add_source`, `update_source`, `remove_source`, `get_preferences`,
  `update_preferences`, `list_assets`, `get_asset_config`,
  `update_asset_config`, `delete_asset`, `validate_asset_config`,
  `create_field2d_config`, `create_field3d_config`, `create_robot_config`,
  `create_joystick_config`.

### Workflow

1. Use `get_layout` to read the current hub/tab structure before making any
   changes.
2. Use `get_tab_type_schema` with the tab type ID to discover valid source
   types, options, and section names before adding sources.
3. Use `add_tab` to create a new tab (type IDs: 0=Documentation,
   1=LineGraph, 2=Field2d, 3=Field3d, 4=Table, 5=Console, 6=Statistics,
   7=Video, 8=Joysticks, 9=Swerve, 10=Mechanism, 11=Points, 12=Metadata).
4. Use `add_source` to populate the tab with data sources. For LineGraph
   tabs, specify the `section` parameter (`leftSources`, `rightSources`, or
   `discreteSources`).
5. Use `get_tab` to verify the final tab configuration.
6. Use `validate_asset_config` when creating or modifying custom assets.

### Conventions

- Source `log_key` paths follow the AdvantageKit convention:
  `NT:/AdvantageKit/<ClassName>/<prefix>/<field>` for `processInputs` data and
  `NT:/AdvantageKit/RealOutputs/<ClassName>/<key>` for `recordOutput` data.
- SmartDashboard-published values use the path
  `NT:/SmartDashboard/<SubsystemName>/<Key>`.
- Colors in source options are hex strings (e.g., `#2b66a2`).
- LineGraph sources use types `stepped` (discrete values), `smooth`
  (continuous values), or `stripes` (boolean discrete bands).
- Tab titles should match the Elastic Dashboard tab names when they show the
  same data to keep navigation consistent across tools.
