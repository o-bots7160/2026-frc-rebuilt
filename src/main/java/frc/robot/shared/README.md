# Shared framework

The `shared/` package contains the cross-cutting base classes, utilities, and
abstractions that every subsystem in the robot builds on. Understanding these
classes is essential before writing or modifying any mechanism — they define the
patterns every subsystem follows.

For a glossary of robotics and programming terms referenced below, see the
[project glossary](../GLOSSARY.md).

## What is in here?

| Folder        | Purpose                                                                                                 |
| ------------- | ------------------------------------------------------------------------------------------------------- |
| `subsystems/` | Abstract subsystem base classes, profiled-motion helpers, and the `VisionMeasurementConsumer` interface |
| `commands/`   | Abstract command and command-factory base classes                                                       |
| `config/`     | Abstract config classes, JSON loading, `SubsystemsConfig`, and `RobotEnvironment`                       |
| `logging/`    | AdvantageKit logger wrapper scoped to each subsystem                                                    |
| `bindings/`   | Trigger and input-binding helpers for controller buttons                                                |

## The motor subsystem hierarchy

All motor-driven mechanisms share a three-tier base-class hierarchy:

```
AbstractSubsystem
  └─ AbstractMotorSubsystem            (motor, feedforward, SysId)
       ├─ AbstractSetAndSeekSubsystem   (position profiling — turret, etc.)
       └─ AbstractVelocitySubsystem     (velocity control — shooter, etc.)
```

`AbstractMotorSubsystem` owns the `Motor` instance, motor inputs, feedforward
model, and SysId routine. It refreshes motor config and feedforward gains each
cycle (when not FMS-attached) and provides `applyVoltage()`, `stop()`, and
telemetry helpers that both branches inherit.

## The set-and-seek pattern

Many mechanisms on an FRC robot follow the same workflow: receive a target
position, plan a smooth path to it, and drive a motor until the target is
reached. We call this **set-and-seek** — you _set_ a goal and the subsystem
_seeks_ it using a
[trapezoidal motion profile](../GLOSSARY.md#trapezoidal-motion-profile).

The turret is the canonical example. See the
[turret README](../subsystems/turret/README.md) for a full walkthrough of how
the pattern works with real hardware.

### How it flows

```
Command calls setTarget(degrees)
        │
        ▼
AbstractSetAndSeekSubsystem
  ├─ clamps to min/max limits
  ├─ converts degrees → radians
  └─ stores as the trapezoid goal
        │
Command calls seekTarget() each cycle
        │
        ▼
AbstractSetAndSeekSubsystem
  ├─ steps the trapezoid profile forward
  ├─ PID computes correction from setpoint vs. measured position
  ├─ feedforward estimates the expected voltage
  └─ sends combined output to the motor
        │
        ▼
Motor wrapper applies the voltage
        │
Command checks isProfileSettled()
  └─ true when position and velocity are within tolerance of the goal
```

## The velocity pattern

Flywheel-type mechanisms (shooter, etc.) follow a different workflow: receive a
target velocity in RPM, ramp to it, and hold it steady. The subsystem reports
"at target" once the measured RPM stays within tolerance for a configurable
settle time.

### How it flows

```
Command calls setTargetVelocityRpm(rpm)
        │
        ▼
AbstractVelocitySubsystem
  ├─ converts RPM → radians/sec
  ├─ optionally steps a trapezoid profile for acceleration limiting
  └─ stores as the velocity goal
        │
Command calls seekVelocity() each cycle
        │
        ▼
AbstractVelocitySubsystem
  ├─ reads motor inputs (position, velocity)
  ├─ PID computes correction from setpoint vs. measured velocity
  ├─ feedforward estimates the expected voltage for the target speed
  └─ sends combined output to the motor
        │
        ▼
Motor wrapper applies the voltage
        │
Command checks isAtTargetVelocity()
  └─ true when |actual − target| < tolerance for settleTimeSeconds
```

### Units convention

The public API uses RPM, which always refers to the **mechanism (flywheel)
speed** — not the motor shaft speed. The gear ratio in the motor config handles
the conversion. Internally, all WPILib math (PID, feedforward, profiles) runs in
radians per second.

## Base classes reference

### Subsystem base classes (`subsystems/`)

| Class                         | Extends                  | Purpose                                                                                                                                                                                                                                                                                                                                                                      |
| ----------------------------- | ------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `AbstractSubsystem`           | WPILib `SubsystemBase`   | Shared foundation for every subsystem. Provides the config reference, a scoped logger, enable/disable gating, simulation awareness, and Driver Station error/warning helpers. Every subsystem constructor receives a config object and inherits `isSubsystemDisabled()` and `logDisabled()` for safety guards.                                                               |
| `AbstractMotorSubsystem`      | `AbstractSubsystem`      | Owns a `Motor`, feedforward, and SysId routine shared by both position and velocity mechanisms. Refreshes motor config and feedforward gains when not FMS-attached. Provides `applyVoltage()`, `stop()`, and position/velocity accessors in radians. Subclasses override `stop()` to reset controller state before halting the motor.                                        |
| `AbstractSetAndSeekSubsystem` | `AbstractMotorSubsystem` | Adds a [trapezoidal motion profile](../GLOSSARY.md#trapezoidal-motion-profile), a [profiled PID](../GLOSSARY.md#profiled-pid) controller, and position tracking. Exposes `setTarget()`, `seekTarget()`, `isProfileSettled()`, `retargetFromCurrent()`. Refreshes gains from tunables when not FMS-attached so you can tune live in the pits.                                 |
| `AbstractVelocitySubsystem`   | `AbstractMotorSubsystem` | Adds a PID velocity controller with optional trapezoidal acceleration limiting. Exposes `setTargetVelocityRpm()`, `seekVelocity()`, `isAtTargetVelocity()`. Public API uses RPM (mechanism speed, not motor shaft speed); internal math uses radians per second. Timer-based settle detection ensures the target is held for a configurable duration before reporting ready. |
| `SysIdHelper`                 | —                        | Static factory that builds a WPILib `SysIdRoutine` for characterizing a single motor. Used by command factories to expose [SysId](../GLOSSARY.md#sysid) commands.                                                                                                                                                                                                            |

### Command base classes (`commands/`)

| Class                         | Extends                    | Purpose                                                                                                                                                                                                                                                                                        |
| ----------------------------- | -------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `AbstractSubsystemCommand`    | WPILib `Command`           | Ties a command to one subsystem, auto-registers the requirement, and logs when the command starts. Subclasses override lifecycle hooks.                                                                                                                                                        |
| `AbstractSetAndSeekCommand`   | `AbstractSubsystemCommand` | Drives a set-and-seek subsystem. On initialize, calls `setTarget()` from a supplier. On execute, calls `seekTarget()`. Finishes when `isProfileSettled()` returns true. On interrupt, schedules a `SetAndSeekSettleCommand` so the mechanism decelerates safely instead of stopping instantly. |
| `SetAndSeekSettleCommand`     | `AbstractSubsystemCommand` | Deceleration command that runs after an `AbstractSetAndSeekCommand` is interrupted. Calls `retargetFromCurrent()` to bleed off velocity smoothly within a timeout.                                                                                                                             |
| `AbstractVelocityCommand`     | `AbstractSubsystemCommand` | Drives a velocity subsystem to a target RPM from a supplier. On initialize, sets the target; on execute, calls `seekVelocity()`. Finishes when `isAtTargetVelocity()` returns true. On end, reverts to idle RPM or stops the motor depending on the interruption flag.                         |
| `AbstractIdleVelocityCommand` | `AbstractSubsystemCommand` | Default command that holds a velocity subsystem at its configured idle RPM. Never finishes on its own — runs until interrupted by a higher-priority command.                                                                                                                                   |

### Command factory base classes (`commands/`)

| Class                              | Extends                           | Purpose                                                                                                                                                                       |
| ---------------------------------- | --------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `AbstractSubsystemCommandFactory`  | —                                 | Base factory holding a subsystem reference. Each subsystem's `commands/` folder has a concrete factory that extends this to keep command creation out of the subsystem class. |
| `AbstractSetAndSeekCommandFactory` | `AbstractSubsystemCommandFactory` | Adds SysId command builders (quasistatic and dynamic, forward and reverse, with configurable timeouts) so every profiled mechanism gets characterization commands for free.   |
| `AbstractVelocityCommandFactory`   | `AbstractSubsystemCommandFactory` | Adds SysId command builders for velocity mechanisms, mirroring the set-and-seek factory pattern.                                                                              |

### Config base classes (`config/`)

| Class                               | Extends                        | Purpose                                                                                                                                                                                                                                                                                                                                                                              |
| ----------------------------------- | ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `AbstractConfig`                    | —                              | Base for all config bundles. Provides `enabled` and `verbose` flags, and [tunable](../GLOSSARY.md#tunable) readers (`readTunableNumber`, `readTunableBoolean`, `readTunableString`, `readTunableDegrees`, `readTunableDegreesAsRadians`) backed by [AdvantageKit](../GLOSSARY.md#advantagekit). When the robot is FMS-attached, tunables short-circuit to their defaults for safety. |
| `AbstractMotorSubsystemConfig`      | `AbstractConfig`               | Shared PID (kP, kI, kD) and feedforward (kS, kV, kA) gains with tunable getters. Used by both set-and-seek and velocity subsystem configs.                                                                                                                                                                                                                                           |
| `AbstractSetAndSeekSubsystemConfig` | `AbstractMotorSubsystemConfig` | Adds fields for setpoint limits, velocity/acceleration limits, position/velocity tolerances, initial state — all stored in degrees with radian getters. Every value is live-tunable on the dashboard.                                                                                                                                                                                |
| `AbstractVelocitySubsystemConfig`   | `AbstractMotorSubsystemConfig` | Adds RPM velocity/acceleration limits, velocity tolerance, settle time, and idle RPM. Public API in RPM with internal rad/s conversion helpers.                                                                                                                                                                                                                                      |
| `AbstractMotorConfig`               | `AbstractConfig`               | Motor-level config: [CAN](../GLOSSARY.md#can-bus) ID, inversion, current limits, [gear ratio](../GLOSSARY.md#gear-ratio), and [soft limits](../GLOSSARY.md#soft-limit) in degrees.                                                                                                                                                                                                   |
| `ConfigurationLoader`               | —                              | Reads JSON config files from the `deploy/` folder and deserializes them into config objects.                                                                                                                                                                                                                                                                                         |
| `SubsystemsConfig`                  | —                              | Top-level config class that holds one config object per subsystem. Loaded from `subsystems.json` (or `subsystems-sim.json` / `subsystems-test.json`).                                                                                                                                                                                                                                |
| `FieldLayoutConfig`                 | —                              | Supplies the AprilTag field layout used by vision and robot state.                                                                                                                                                                                                                                                                                                                   |

### Disabled-subsystem lifecycle

When a subsystem's `enabled` flag is `false` in `subsystems.json`:

1. The config is loaded normally and `enabled = false` is read.
2. `AbstractSubsystem` copies the flag; `isSubsystemDisabled()` returns `true`.
3. The subsystem constructor bails out early, skipping hardware init.
4. Motor-backed subsystems use `DisabledMotor` (a no-op motor) so callers never
   need null checks.
5. `RobotContainer` still constructs the subsystem — it becomes inert, keeping
   wiring simple and avoiding null references.
6. Every public method that mutates state checks `isSubsystemDisabled()` first
   and returns early, calling `logDisabled("methodName")` so operators can see
   the skipped call in telemetry.

## Adding a new shared abstraction

Before adding a new base class here, ask: "Will at least two subsystems use
this?" If the answer is no, keep the code in the mechanism folder until a second
user appears. When it does belong here, add Javadoc with a one-sentence summary,
usage guidance, and `@param`/`@return` tags so students can understand the class
from the docs alone.
