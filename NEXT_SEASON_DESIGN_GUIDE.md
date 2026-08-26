# Design Considerations and Improvements for Next Season

## Purpose

This document is a game-agnostic reference for initializing and developing a
new FRC Java robot repository. Use it after the annual game reveal, together
with the official game manual and the current WPILib documentation.

The game, robot design, mechanisms, sensors, and scoring strategy are
intentionally unknown. Do not copy this season's subsystem list into the new
repository. Apply the architecture and development practices in this guide only
after the team has defined the new robot's requirements.

The goals are to:

- preserve the development patterns that worked well;
- remove known sources of latency, unnecessary allocation, and coupling;
- use smart motor controllers for local, high-rate control;
- support real hardware, simulation, and deterministic log replay from the
  beginning;
- keep the code understandable for students;
- make performance and safety measurable rather than assumed; and
- avoid building abstractions or mechanisms before the game requires them.

## How to use this guide

An agent or developer initializing the next repository should treat this
document as an architecture contract. The official game manual remains the
authority for robot rules, field geometry, timing, and legal hardware.

A suitable initialization prompt is:

> Initialize this empty Git repository for the 2027 FRC game `{GAME_NAME}`.
> Prepare it to build the required subsystems, commands, IO implementations,
> configuration, tests, simulation, logging, and utilities. Follow the
> guidelines and standards in `{PATH_TO_THIS_DOCUMENT}` and comply with the
> official FRC game manual at `{PATH_OR_URL_TO_GAME_MANUAL}`. Do not invent
> mechanisms or requirements that have not been approved. Record assumptions
> and unresolved rule questions explicitly.

Before generating mechanism code, provide the agent with:

1. the current game manual and relevant team updates;
2. an approved list of robot capabilities and mechanism responsibilities;
3. known hardware selections and sensor choices;
4. coordinate frames, units, and field landmarks;
5. safety constraints and cross-mechanism collision rules; and
6. acceptance criteria for autonomous and teleoperated behavior.

## Guiding principles

### Build only what the game requires

- Start with infrastructure, not placeholder mechanisms.
- Create a subsystem only after its responsibility, inputs, outputs, sensors,
  and safety constraints are understood.
- Keep game strategy separate from reusable hardware and control code.
- Prefer small, replaceable interfaces over speculative inheritance
  hierarchies.
- Reassess abstractions after two concrete uses. Do not generalize from one
  hypothetical mechanism.

### Keep the roboRIO responsible for decisions

The roboRIO should:

- run the command scheduler and robot state machines;
- select mechanism goals;
- coordinate mechanisms and collision constraints;
- calculate field-relative targets and autonomous behavior;
- validate sensor state and enforce robot-level safety; and
- capture one coherent input snapshot per loop.

Smart motor controllers should normally:

- run local position, velocity, or current loops;
- generate supported local motion profiles;
- apply supported feedforward terms;
- enforce current limits and motor-local limits; and
- mirror leader output through hardware follower modes.

This division gives local motor control a faster and more deterministic update
rate while keeping robot behavior visible and testable on the roboRIO.

### Measure before optimizing

- Treat the default 20 ms robot loop as a deadline, not a target average.
- Record timing from the first deploy.
- Optimize measured hotspots before adding threads or replacing libraries.
- Never increase watchdog periods merely to hide overruns.
- Validate changes under realistic CAN traffic, camera load, dashboard
  connections, and logging.
- Require performance changes to include before-and-after worst-case timing,
  allocation, or CAN evidence. A commit description that says "performance" is
  not evidence.

## Practices to continue

The following patterns worked well and should remain part of the next design.

### Use `RobotContainer` as the composition root

- Load configuration first.
- Construct subsystems in dependency order.
- Wire cross-subsystem data using suppliers, consumers, or narrow interfaces.
- Construct command factories after subsystems.
- Assign default commands after factories.
- Configure controller bindings last.
- Keep concrete dependency construction out of subsystems and commands.

This keeps object ownership and robot wiring visible in one place.

### Separate commands from subsystems

- Subsystems own state, hardware-facing operations, safety checks, and
  telemetry.
- Commands express when operations run and what requirements they hold.
- Command factories live beside the commands for their mechanism.
- Subsystems must not manufacture commands.
- Long-running commands must define interruption behavior explicitly.

### Decouple subsystems

- Do not store a direct reference to one subsystem inside another subsystem.
- Pass only the capability that is needed, such as `Supplier<Pose2d>`,
  `DoubleSupplier`, `Consumer<State>`, or a small domain interface.
- Wire concrete dependencies in `RobotContainer`.
- Keep one authoritative source for shared state such as robot pose or gameplay
  mode.

### Use environment-specific configuration

- Support competition, test, and simulation hardware without conditional logic
  spread throughout the codebase.
- Keep CAN IDs, inversion, gearing, physical dimensions, current limits, and
  mechanism constraints in configuration.
- Store values in human-readable units and convert at clear boundaries.
- Keep tunable values visible in logs so adjustments can be reproduced.
- Persist approved tuned values back to source-controlled configuration.
- Select competition and test hardware from an explicit deploy-time identity or
  validated hardware fingerprint. Never use a checked-in boolean to choose the
  robot.
- Reject unknown or misspelled competition configuration fields and validate
  required values and ranges before constructing hardware.
- Configuration errors must identify the file and property path and preserve
  the original exception cause.

### Preserve disabled-subsystem behavior

- Construct every configured subsystem even when it is disabled.
- Use no-op IO implementations instead of `null`.
- Skip hardware initialization when disabled.
- Guard public mutating methods and report skipped actions once.
- Disable and degrade gracefully when hardware initialization fails.

### Continue startup preloading

Perform predictable, potentially expensive initialization before the robot is
enabled:

- parse deploy configuration;
- configure hardware;
- load field layouts;
- register named commands;
- preload autonomous routines;
- warm pathfinding or other known first-use Java paths; and
- publish required dashboard controls.

Do not parse files, persist controller configuration, or discover autonomous
assets in periodic code.

Only publish autonomous routines that loaded and validated successfully.
Quarantine malformed assets, provide an explicit safe "No Auto" option, and
never fall back to synchronous auto parsing when autonomous is enabled.

### Continue deliberate telemetry control

- Use AdvantageKit as the primary diagnostic and replay log.
- Use dashboards only for operator-critical controls and status.
- Keep verbose telemetry disabled by default.
- Guard expensive visualization construction before calling a verbose logging
  method.
- Reduce or disable telemetry built into third-party libraries when the same
  data is already captured elsewhere.
- Give each signal one canonical log key. Do not record an AutoLogged input
  again as a separate output unless the second value is transformed or serves a
  documented operator purpose.

### Continue bounded vision processing

- Run image processing on a coprocessor.
- Put an explicit limit on the number of unread camera results processed during
  one robot cycle.
- Prefer the newest useful observations when a backlog exists.
- Timestamp every observation at capture time.
- Reject physically impossible or low-confidence observations before fusion.
- Log accepted and rejected observations with clear reason counts.

### Development practices confirmed by change history

The repository history included several approaches that were added, adjusted,
removed, and later reintroduced in narrower forms. Carry these process lessons
forward:

- Review the actual diff and acceptance evidence, not only the commit subject.
- Keep tuning changes separate from architecture and correctness changes.
- Do not carry prior-season gains, limits, camera transforms, tag IDs, field
  landmarks, or mechanism dimensions into a new game.
- Introduce abstractions after concrete requirements reveal the shared
  capability.
- Define one owner for every state, unit conversion, update loop, and reset.
- Add a regression test when a real robot failure teaches a reusable lesson.
- Replace temporary diagnostics with measured instrumentation, then remove the
  temporary code.
- Document why a methodology changed so a later cleanup does not accidentally
  restore the original failure.
- Use short architecture decision records when the team reverses or materially
  narrows an earlier design.

## Required architectural improvements

### Define explicit runtime modes

Create an explicit runtime mode enum:

```java
public enum RobotMode {
    REAL,
    SIM,
    REPLAY
}
```

Select the mode once during startup. Do not infer replay from
`RobotBase.isReal()`.

Select robot identity separately from runtime mode. Runtime mode answers whether
the process is real, simulated, or replaying; robot identity selects the
competition, practice, or test hardware configuration.

Each subsystem should receive one of:

- real hardware IO in `REAL`;
- physics-backed simulation IO in `SIM`; or
- no-op IO whose inputs are populated by AdvantageKit in `REPLAY`.

Replay must not construct hardware devices or advance independent physics
models.

### Use one input snapshot per cycle

Every hardware-owning subsystem should follow this sequence:

```java
io.updateInputs(inputs);
Logger.processInputs("Subsystem", inputs);
```

After that point, all control, readiness, safety, and telemetry calculations in
the current cycle must read `inputs`. Do not call vendor getters again from:

- commands;
- controller calculations;
- readiness checks;
- logging helpers; or
- public subsystem accessors.

Benefits include:

- coherent sensor values throughout the cycle;
- deterministic replay;
- fewer repeated JNI or vendor-library calls;
- simpler timing measurement; and
- clearer ownership of stale or disconnected data.

### Prefer capability-oriented IO

Do not force every mechanism through a voltage-only motor abstraction. Define
interfaces around mechanism capabilities and keep vendor types private to real
IO implementations.

Possible capabilities include:

```java
interface VelocityMechanismIO {
    void updateInputs(VelocityMechanismIOInputs inputs);

    void setVelocity(double velocityRadiansPerSecond, double feedforwardVolts);

    void setVoltage(double volts);

    void stop();
}
```

```java
interface ProfiledPositionMechanismIO {
    void updateInputs(ProfiledPositionMechanismIOInputs inputs);

    void setProfiledPosition(double positionRadians);

    void setEncoderPosition(double positionRadians);

    void setVoltage(double volts);

    void stop();
}
```

Create only the capabilities required by approved mechanisms. Keep SysId
voltage control available even when normal operation uses onboard closed-loop
control.

### Define one owner for units and conversion

For every sensor and command value:

- define the public mechanism unit;
- define the raw controller unit;
- apply the gear ratio and offset at exactly one boundary;
- test both forward and inverse conversion;
- log raw units, mechanism units, and the applied ratio when diagnosing setup;
  and
- confirm the units expected by controller soft limits, profiles, feedforward,
  simulation, and SysId.

If Java owns conversion, explicitly reset persisted firmware conversion factors
to identity values. If firmware owns conversion, do not repeat the conversion in
Java.

Treat SysId units as a versioned interface. Record the WPILib/SysId version and
validate analyzer output with a known conversion before applying gains. Do not
add or remove `2pi` corrections based only on an assumption about the tool.

### Use smart-controller features intentionally

For REV SPARK controllers, evaluate these features for each mechanism:

- onboard position, velocity, or current PID;
- MAXMotion position profiling;
- MAXMotion velocity acceleration limiting;
- onboard static, velocity, acceleration, elevator-gravity, or cosine-arm
  feedforward;
- closed-loop slots for different operating modes;
- hardware limit switches and soft limits;
- hardware follower mode;
- open-loop and closed-loop ramp rates;
- smart current limits; and
- controller-reported at-setpoint and fault status.

The roboRIO must still clamp targets, coordinate mechanisms, and enforce
robot-level interlocks.

Do not copy roboRIO PID gains directly into a controller. Confirm units,
feedback sensor direction, conversion factors, update rates, and feedforward
definitions, then retune the controller.

Use roboRIO-generated profiles when the controller cannot represent the
required motion, such as:

- synchronized multi-axis motion;
- asymmetric acceleration and deceleration;
- jerk-limited trajectories;
- dynamic waypoints;
- rapidly changing constraints; or
- nonlinear models not supported by controller feedforward.

Choose one voltage-command contract. When using WPILib `setVoltage`, do not also
enable controller voltage compensation unless measured testing demonstrates
that the combined behavior is intentional. Validate commanded voltage, applied
voltage, and behavior under battery sag.

Use vendor hardware follower mode for mechanically coupled motors unless
independent control is required. Log current, temperature, faults, connectivity,
and applied output for every follower; primary-only telemetry cannot detect a
failed follower.

### Preserve motion-profile continuity

- Repeated identical goals must not restart a profile.
- Document whether each target supplier is sampled once during `initialize()` or
  continuously during `execute()`.
- Use continuous sampling only for intentionally live goals and ignore changes
  below a meaningful tolerance.
- When retargeting or settling during motion, seed profile state from the latest
  measured position and velocity.
- For continuously changing goals, use tolerance, hysteresis, or a readiness
  latch rather than resetting a settle timer for every numerical change.

### Apply configuration only when values change

Do not rebuild controllers, profiles, feedforward objects, or dashboard keys
every 20 ms.

- Construct stable dashboard keys once.
- Cache the last applied tunable values.
- Apply gains and constraints only after detecting a change.
- Do not persist motor configuration from periodic code.
- Keep startup configuration synchronous and validated.
- Use asynchronous configuration only when ordering, completion, and failure
  behavior are explicit.

### Validate hardware configuration

- Check every vendor configuration result.
- Retry startup configuration a bounded number of times where appropriate.
- Report the device name, CAN ID, attempted operation, and final error.
- Set the owning subsystem to a safe disabled state after a fatal failure.
- Record firmware and reset faults.
- Avoid stale settings by applying a complete code-owned configuration at
  startup.
- Never call an overridable hardware-configuration method from a base-class
  constructor. Construct subclass state first, then invoke an explicit,
  idempotent initialization step.

## Drivetrain library strategy

### Default decision

Keep YAGSL as the initial swerve implementation unless profiling or game
requirements demonstrate a reason to replace it.

YAGSL provides substantial value:

- motor, encoder, and gyro integration;
- module optimization and encoder synchronization;
- controller configuration and closed-loop module control;
- kinematics and pose estimation;
- vision measurement integration;
- simulation;
- JSON-based hardware configuration; and
- maintained support for vendor changes.

A custom implementation would still need to perform the essential sensor,
kinematics, pose-estimation, and motor-command work. Removing library
abstraction alone is unlikely to produce a major runtime improvement.

### Keep YAGSL replaceable

Hide YAGSL behind a small drivetrain IO boundary. Commands, autonomous code,
pose consumers, and other subsystems must not depend on `SwerveDrive`,
`SwerveModule`, or other YAGSL types.

The drivetrain subsystem itself should also depend only on `DriveBaseIO`.
Construction, getters, commands, resets, and vision insertion on
`SwerveDrive` belong inside the YAGSL IO implementation.

The drivetrain boundary should expose only approved capabilities, such as:

- cached timestamped odometry samples;
- measured module states and positions;
- robot-relative chassis velocity;
- field pose;
- robot-relative chassis commands;
- pose reset;
- vision measurement insertion;
- brake/coast selection; and
- drivetrain health.

### Optimize the integration before replacing the library

- Configure YAGSL once during startup.
- Use closed-loop module control unless testing demonstrates a need for
  open-loop control.
- Set YAGSL telemetry to the lowest useful level.
- Capture module states and positions once per sample.
- Avoid maintaining duplicate odometry estimators unless the diagnostic value
  justifies the cost.
- Configure an appropriate odometry period and verify timestamp quality.
- Measure YAGSL update time, CAN utilization, allocation rate, and signal age.
- Do not manually invoke a library update already owned by its odometry thread
  or periodic lifecycle.
- Never catch `NullPointerException` as a device-readiness signal. Correct
  initialization ownership or return explicit health state.

Consider a custom drivetrain implementation only when:

- repeated measurements identify YAGSL as a material deadline contributor;
- the game requires unsupported drivetrain behavior;
- required hardware or sensors are unsupported;
- timestamp or high-rate odometry requirements cannot be satisfied; or
- library behavior prevents a required safety or reliability guarantee.

Document the evidence and expected improvement before approving a rewrite.

## Loop-time and performance standards

### Record these metrics

At minimum, log:

- full-cycle duration;
- user-code duration;
- logger periodic duration;
- garbage-collection duration and count;
- logger receiver queue depth;
- command scheduler duration;
- each subsystem input-update duration;
- expensive command execution duration;
- CAN utilization and error counts;
- disconnected or stale devices;
- vision result count and discarded backlog; and
- active command names or lifecycle events.

Use monotonic time for profiling. AdvantageKit may intentionally return one
deterministic FPGA timestamp throughout a cycle, so ordinary FPGA time is not
appropriate for measuring work inside that cycle.

### Performance acceptance

- Every full robot cycle must finish before the configured loop period.
- Preserve practical headroom below 20 ms under worst-case expected load.
- Do not approve performance based only on average duration.
- Test with all expected cameras, controllers, dashboards, logs, and CAN
  devices active.
- Run long-duration tests to reveal garbage collection and queue growth.
- Store representative performance logs for regression comparison.

### Avoid common allocation sources

In periodic and command execution paths:

- reuse input containers and large collections;
- avoid streams when a simple loop avoids temporary objects;
- avoid per-cycle string formatting;
- rate-limit large pose arrays and mechanism visualizations;
- avoid rebuilding immutable controller objects without a value change;
- avoid constructing commands dynamically every cycle; and
- correlate allocation changes with garbage-collection metrics before
  performing micro-optimizations.

Small geometry objects are acceptable when measurements show adequate
headroom. Readability remains important.

## CAN bus standards

Create a signal budget before enabling every telemetry getter.

- Assign fast rates only to signals used for control or odometry.
- Assign slower rates to temperature, faults, and other diagnostics.
- Disable signals that are not consumed.
- Account for signals that share a status frame.
- Preserve the update rate required by hardware follower behavior.
- Batch compatible signal refreshes when the vendor API supports it.
- Cache each signal once for the robot cycle.
- Monitor stale data, timeouts, and bus utilization on the actual robot.

There is no universal safe utilization percentage for every topology. Validate
the complete bus under realistic load.

## Localization, vision, and targeting standards

### Keep one pose-estimation owner

- Exactly one estimator owns the fused robot pose.
- Vision IO emits timestamped measurements and uncertainty; it does not maintain
  or write back a second fused estimate.
- Gameplay code consumes the authoritative fused pose.
- Odometry-only pose may be exposed separately for diagnostics.
- A diagnostic estimator must have measured value, a bounded update cost, and a
  clear removal or disable path.

### Define pose lifecycle and reset ownership

Model these as explicit states or events:

- initial pose acquisition;
- disabled calibration;
- normal fusion;
- manual reset; and
- recovery from large odometry disagreement.

Choose exactly one owner for autonomous pose reset. Validate all deployed
autonomous assets against that policy; do not mix robot-code resets with
per-auto `resetOdom` behavior.

Do not reset pose at teleop entry after autonomous unless a validity check
requires it. Every hard reset must:

- reset the owning estimator atomically;
- invalidate temporal filter and targeting warm-start state;
- record the reset source;
- require a fresh, validated measurement when vision initiated it; and
- be tested across disabled-to-auto and auto-to-teleop transitions.

Manual vision reset must enforce configurable measurement age, connection,
tag-count, ambiguity, and confidence requirements.

### Keep simulation truth independent

Simulated cameras and sensors must derive observations from independent
ground-truth simulation state. Never generate simulated measurements from the
estimator that will consume those measurements.

### Validate camera configuration

- Give every camera a stable network identity and independent enable flag.
- Store transforms in meters and radians with documented coordinate frames.
- Validate transforms for finite and physically plausible values at startup.
- Pin compatible coprocessor software and robot-side library versions.
- Run a repeatable transform, latency, disconnect, and reconnect acceptance
  test before enabling fusion.

### Make filtering explicit

- Represent filtering as ordered, reason-coded checks.
- Normalize vendor sentinel values at the IO boundary.
- Remove disallowed targets before solving a pose; averages must not hide one
  invalid or distant observation.
- Scale uncertainty using observed geometry and tag count.
- Allow per-axis uncertainty when one dimension is less trustworthy.
- Distinguish estimator acceptance from operator visibility hysteresis.
- Normalize missing replay collections to empty collections at the IO boundary.
- Test every rejection reason and threshold boundary.

Large vision-to-odometry disagreements may bypass the normal gate only after
multiple fresh, consecutive, mutually consistent multi-tag observations.
Define maximum inter-frame age and total recovery duration. Clear recovery
candidates on timeout, disconnect, reset, calibration transition,
non-qualifying observation, or camera change.

### Keep field and targeting math pure

- Keep field landmarks in one source-controlled field model.
- Implement alliance transformations as tested pure functions.
- Treat an unknown alliance as unavailable for alliance-dependent automation;
  do not silently assume a color.
- Keep ballistic and shoot-on-the-move calculations independent of subsystems.
- Validate all solver inputs as finite and within expected ranges.
- Account for the launcher's offset and rotational velocity when required.
- Bound iterative solvers and log convergence or fallback.
- Invalidate warm-start state after pose reset, target change, alliance change,
  or stale input.
- Treat visual simulation as a diagnostic, not proof of physical accuracy.

## Threading rules

Keep command scheduling, subsystem state, and robot coordination on the main
thread.

Use a worker or `Notifier` only for work that is:

- genuinely blocking;
- required at a higher sample rate;
- isolated inside an IO implementation; and
- proven to threaten the main loop.

Examples may include timestamped high-rate odometry or blocking sensor waits.

Every worker must:

- use bounded queues or a synchronized snapshot;
- timestamp samples at acquisition;
- define data freshness and ownership;
- handle exceptions without silently dying;
- complete within its own period;
- avoid scheduling commands;
- avoid mutating subsystem state directly; and
- never call AdvantageKit logging APIs off the main thread.

`TimedRobot.addPeriodic()` runs synchronously with the robot loop and is not a
background worker.

## Commands and state coordination

### Prefer semantic goals

Commands and high-level coordinators should request goals such as:

- acquire;
- hold;
- score;
- travel;
- home; or
- stop.

Do not expose vendor control modes or raw motor voltages to gameplay code.

### Separate mechanism control from coordination

- Each subsystem owns one mechanism and its local safety.
- A coordinator or command composition owns sequencing between mechanisms.
- Collision avoidance belongs in a dedicated planner or coordinator, not in
  controller bindings.
- Manual recovery commands must remain available for operators.
- State transitions must be logged with their source.

A gameplay-state value and active mechanism behavior must transition
atomically. Periodic or FMS logic may request an event, but the composition layer
must schedule the corresponding coordinator behavior; never update only the
displayed state.

Every command that mutates a subsystem must declare that subsystem as a
requirement. Coordinators must also require a coordinator token or subsystem
when transitions must be mutually exclusive.

### Make command composition semantics explicit

- Factories return fresh command graphs; do not reuse an already composed
  command instance.
- Document the controlling child of every `parallel`, `deadline`, and `race`
  group.
- Register named commands before loading autonomous routines.
- Use proxies only when delayed requirement acquisition is intentional and
  document that the parent cannot see proxy requirements.
- Use deferred commands when alliance, pose, chooser, or tunable values must be
  resolved at schedule time rather than repository startup.
- Treat readiness as temporal behavior. Define what happens when readiness is
  gained, lost briefly, and regained.

### Treat interruptions as normal behavior

- Do not emit warnings or stack traces when a command ends normally.
- Use warnings for recoverable faults that need operator attention.
- Clear persistent alerts when their conditions recover.
- Define whether interruption stops immediately, holds position, or performs a
  controlled settle.
- Cleanup may stop or settle mechanisms directly, but must not schedule
  replacement commands from `end()` or `finallyDo()`. Let defaults resume,
  compose cleanup into the enclosing command, or bind an explicit release
  transition.

### Match controller bindings to command lifetime

- Use `onTrue` for one-shot actions.
- Use `whileTrue` for hold-to-run behavior.
- Use `toggleOnTrue` only for intentionally latched modes with an obvious
  operator indication and cancellation path.
- Falling-edge debounce delays cancellation. Do not apply it to emergency-stop
  or immediate-release safety controls.
- Input health monitoring belongs in periodic infrastructure, not a default
  command.

## Safety and error handling

- Guard all public mutating subsystem methods when the subsystem is disabled.
- Put hardware limits in the motor controller where possible.
- Also enforce robot-level limits and cross-mechanism interlocks.
- Validate targets for finite values and legal ranges.
- Detect disconnected, stale, reset, and faulted hardware.
- Degrade one failed mechanism without crashing unrelated robot systems.
- Never throw unchecked exceptions from periodic robot code.
- Report fatal initialization failures with stack traces.
- Report recoverable runtime conditions without stack traces.
- Never use console output as high-frequency telemetry.
- Permit command execution while disabled only for explicitly reviewed,
  non-actuating calibration or state operations. Test that no motor output can
  result.
- Interlock-bypass commands must be clearly named, hold-to-run, restricted to
  test or tuning mode, and preserve independent hard safety limits.
- Temporary diagnostics require an owner and removal condition and must not
  emit stack traces for ordinary command interruption.

The official game manual must be reviewed before implementing motion limits,
starting configurations, extension behavior, autonomous actions, field
interaction, or control-system assumptions.

## Simulation and replay

### Simulation

Simulation should:

- use physics-backed IO implementations;
- consume the same semantic goals as real IO;
- model gearing, inertia, limits, current draw, and sensor direction;
- expose mechanism visualization where useful; and
- permit autonomous and command testing before hardware exists.

Simulation gains are starting points only. Retune on the physical mechanism.
Keep controller-simulation state, encoder state, and physics-model state
synchronized whenever position is seeded or reset. Configure physical inertia
independently from motion-profile acceleration constraints; a profile limit is
not a mechanism inertia measurement.

### Replay

Replay should:

- instantiate replay/no-op IO;
- receive recorded inputs through `Logger.processInputs`;
- avoid advancing independent physics;
- avoid hardware access;
- reproduce control and decision outputs from the recorded inputs; and
- run faster than real time when desired.

Use a versioned replay input schema and normalize absent legacy fields at the IO
boundary. Add a replay startup test and missing-field tests before depending on
replay for match diagnosis.

## Testing expectations

Establish unit tests while infrastructure is small.

Test at least:

- unit and gear-ratio conversions;
- setpoint clamping and invalid-value rejection;
- runtime mode selection;
- real/sim/replay IO selection;
- controller configuration generation;
- command requirements, defaults, state transitions, and interruption behavior;
- `parallel`, `deadline`, and `race` completion behavior;
- proxy and deferred-command cancellation;
- readiness loss, hysteresis, and recovery;
- disabled command execution;
- collision and safety constraints;
- pose and coordinate transforms;
- vision filtering, recovery, replay-null handling, and measurement freshness;
- camera transforms and field/alliance transforms;
- autonomous asset loading, chooser contents, and pose-reset policy;
- configuration schema and robot-identity mapping;
- replay behavior using recorded inputs.

For each mechanism added after kickoff:

1. test pure calculations without hardware;
2. test its simulation IO;
3. test command lifecycle and requirements;
4. characterize the real mechanism with SysId where appropriate;
5. verify controller configuration and limits with the robot disabled or
   safely restrained; and
6. record a repeatable acceptance test.

Run the normal build before merging runnable changes. CI must fail when an
expected test module discovers zero tests.

## Build and deployment standards

- Build and deploy tasks must not stage files, create commits, or otherwise
  mutate Git state.
- If an event snapshot commit is useful, provide a separate, explicit
  operator-invoked task.
- Validate deploy configuration and autonomous assets before deleting or
  replacing files on the roboRIO.
- Remove stale deploy assets so deleted paths or configs cannot remain active.
- Report deploy start and finish times and the deployed Git SHA.
- Keep build metadata in logs so every robot run can be traced to source.
- Treat startup dependency failures by capability. For example, a drivetrain
  may retain teleop while autonomous is explicitly disabled, but the degraded
  state must be operator-visible and preserve the original failure cause.

## Suggested repository structure

Create infrastructure folders, but do not add placeholder mechanism classes:

```text
src/main/java/frc/robot/
  Robot.java
  RobotContainer.java
  commands/
  shared/
    commands/
    config/
    field/
    logging/
    runtime/
    subsystems/
  subsystems/
    drivebase/
      commands/
      config/
      io/
src/main/deploy/
src/test/java/frc/robot/
vendordeps/
```

After mechanisms are approved, add:

```text
subsystems/<mechanism>/
  <Mechanism>Subsystem.java
  commands/
  config/
  io/
```

Do not add a separate `factories/` directory. Keep each command factory beside
the commands it creates.

## Repository initialization checklist

An empty next-season repository is ready for mechanism development when it has:

- the current WPILib command-based Java project;
- build metadata;
- explicit `REAL`, `SIM`, and `REPLAY` mode selection;
- AdvantageKit logging and replay startup;
- a documented 20 ms loop-time budget;
- command lifecycle and loop timing instrumentation;
- configuration loading with separate real, test, and simulation values;
- validated competition/test robot identity selection;
- strict configuration schemas with actionable failure messages;
- a no-op disabled IO pattern;
- a drivetrain IO boundary with YAGSL contained in its real implementation;
- PathPlanner integration, asset validation, safe chooser defaults, and startup
  warmup if autonomous planning is approved;
- one documented autonomous pose-reset policy;
- testing and simulation configured;
- CI that runs the normal build and verifies expected tests are discovered;
- replay startup and missing-input-schema tests;
- formatting and static-analysis tools already supported by the chosen WPILib
  project;
- a current architecture diagram;
- a glossary for student-facing domain terms; and
- no speculative game mechanisms.

## Mechanism design checklist

Before generating a subsystem, answer:

1. What single physical responsibility does it own?
2. What semantic goals must callers request?
3. What sensors provide authoritative position or velocity?
4. Which control work belongs on the smart controller?
5. What units cross each API boundary?
6. What are the physical, software, and game-rule limits?
7. How is the mechanism homed or zeroed?
8. What happens when a sensor disconnects or the controller resets?
9. What must happen when its command is interrupted?
10. Which other mechanisms can collide with it?
11. What must be logged for diagnosis?
12. How will it be simulated, replayed, and tested?
13. Where is each raw-to-mechanism unit conversion owned?
14. Is each target supplier sampled once or continuously?
15. What objective test proves it is ready?

Do not generate the subsystem until these questions have approved answers or
explicitly documented assumptions.

## Anti-patterns to avoid

- A universal motor abstraction that exposes only raw voltage and hides useful
  controller capabilities.
- Direct vendor-object access from commands or gameplay code.
- Direct subsystem-to-subsystem references.
- Hardware getters scattered throughout a loop.
- Different sensor values used by logging and control in the same cycle.
- Runtime configuration persistence in periodic code.
- Rebuilding controllers or profiles every cycle.
- Overridable initialization methods called from base-class constructors.
- Multiple components converting the same sensor value.
- Profiles restarted by repeated identical goals.
- Software follower commands with no follower health telemetry.
- Unbounded camera or message backlog processing.
- High-rate formatted strings, stack traces, or console output.
- Persistent alerts that are never cleared.
- Scheduling replacement commands from `end()` or `finallyDo()`.
- Coordinator state changing without its mechanism behavior.
- Debounced release on an immediate-stop safety control.
- Diagnostic interlock bypasses exposed in competition bindings.
- Threads added before profiling.
- Main-thread blocking waits.
- Increasing the loop period to silence overruns.
- Simulation code running during replay.
- Simulated sensors driven from the estimator they feed.
- More than one owner resetting or fusing robot pose.
- Synchronous autonomous parsing when the robot is enabled.
- Unknown configuration fields accepted silently.
- Build or deploy tasks that modify Git state.
- Placeholder subsystems for mechanisms the game may not require.
- Copying prior-season mechanism code without revalidating units, geometry,
  sensors, limits, or game rules.

## Decision record expectations

Record short architecture decisions for consequential choices, including:

- selecting or replacing a drivetrain library;
- choosing onboard versus roboRIO control;
- adding a worker thread;
- selecting pose-estimation ownership;
- selecting autonomous pose-reset ownership;
- introducing a shared abstraction; and
- accepting a measured performance tradeoff.

Each decision should state:

- the requirement;
- options considered;
- measurements or constraints;
- the selected approach;
- risks and fallback; and
- how the decision will be validated.

When a later decision reverses an earlier one, link the records and explain the
new evidence. Do not delete the original context.

## Reference sources

Use current-season versions of these sources during initialization:

- [WPILib documentation](https://docs.wpilib.org/)
- [WPILib command-based programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
- [WPILib Java garbage collection guidance](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-gc.html)
- [AdvantageKit documentation](https://docs.advantagekit.org/)
- [AdvantageKit IO interfaces](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/)
- [AdvantageKit built-in logging and timing](https://docs.advantagekit.org/data-flow/built-in-logging/)
- [REVLib documentation](https://docs.revrobotics.com/revlib/)
- [REV closed-loop control](https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started)
- [REV MAXMotion position control](https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-position-control)
- [REV MAXMotion velocity control](https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-velocity-control)
- [YAGSL documentation](https://docs.yagsl.com/)
- [PathPlanner documentation](https://pathplanner.dev/)

Always confirm that documentation matches the WPILib year and deployed vendor
library versions.
