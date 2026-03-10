# Shared Subsystems

Abstract base classes that every concrete subsystem mechanism extends. These
bases handle the disabled-subsystem lifecycle, motor management, feedforward
estimation, SysId characterization, and motion profiling so concrete subsystems
only need to supply their hardware wiring and mechanism-specific behavior.

## Hierarchy

```
AbstractSubsystem<TConfig>
└── AbstractMotorSubsystem<TConfig>
    ├── AbstractSetAndSeekSubsystem<TConfig>
    └── AbstractVelocitySubsystem<TConfig>
```

## Key classes

| File                               | Role                                                                    |
| ---------------------------------- | ----------------------------------------------------------------------- |
| `AbstractSubsystem.java`           | Base class with config, enabled flag, logging, and sim detection        |
| `AbstractMotorSubsystem.java`      | Adds a `Motor`, feedforward, telemetry input processing, and SysId      |
| `AbstractSetAndSeekSubsystem.java` | Trapezoidal motion profile with `setTarget` / `seekTarget` flow         |
| `AbstractVelocitySubsystem.java`   | Velocity PID + feedforward with `setTargetVelocityRpm` / `seekVelocity` |
| `VisionMeasurementConsumer.java`   | Functional interface for accepting vision pose measurements             |
| `SysIdHelper.java`                 | Factory for building consistent `SysIdRoutine` instances                |

## Patterns

### Set-and-seek

For mechanisms that move to a target position (turret, climber, harvester):

1. `setTarget(degrees)` → clamps to config limits, logs, and stores the goal.
2. `seekTarget()` → steps the trapezoidal profile and applies feedforward + PID
   voltage.
3. `isProfileSettled()` → returns true when position and velocity are within
   tolerance.

See the [turret README](../../subsystems/turret/README.md) for a concrete
example.

### Velocity

For mechanisms that maintain a target speed (shooter, feeder, indexer, intake):

1. `setTargetVelocityRpm(rpm)` → stores the target with optional acceleration
   ramping.
2. `seekVelocity()` → applies feedforward + PID voltage to track the RPM
   setpoint.
3. `isAtTargetVelocity()` → returns true when the motor speed is within the
   configured tolerance for the settle time.

### SysId characterization

Both motor subsystem types support WPILib SysId via `SysIdHelper`. The command
factories expose quasistatic and dynamic characterization commands. After
running SysId, use the gains (applying the 2π correction) in the subsystem
config. See [SYSID_GUIDE.md](SYSID_GUIDE.md) for the full workflow.

### Disabled-subsystem lifecycle

All subsystems inherit the `enabled` flag from `AbstractConfig`. When disabled:

- `isSubsystemDisabled()` returns true.
- Public methods return early after calling `logDisabled("methodName")`.
- Motor-backed subsystems use `DisabledMotor` so callers never need null checks.
- `RobotContainer` still constructs every subsystem; they just become inert.
