# Shared Configuration

Handles loading, deserializing, and live-tuning of subsystem configuration from
JSON deploy files. Every subsystem's config class extends one of the abstract
bases in this package, inheriting the tunable-value system and the enabled flag.

## How configuration loading works

1. `ConfigurationLoader.load(filename, class)` reads a JSON file from
   `src/main/deploy/` using Jackson and returns a deserialized config object.
2. After deserialization, the loader records a read-only snapshot of every field
   to AdvantageKit under `Config/<className>/...` for replay debugging.
3. Tunable values (numbers, booleans, strings) are managed separately by
   `AbstractConfig`. Each getter (e.g., `getkP()`) reads the live value from an
   AdvantageKit-backed `LoggedNetworkNumber` on SmartDashboard so operators can
   tweak gains on-robot without redeploying.
4. When FMS is attached, tunable values are not created to reduce NetworkTables
   noise.

## Config hierarchy

```
AbstractConfig
├── AbstractSubsystemConfig
│   └── AbstractMotorSubsystemConfig
│       ├── AbstractSetAndSeekSubsystemConfig
│       └── AbstractVelocitySubsystemConfig
├── MotorConfig
└── (concrete subsystem configs)
```

## Key classes

| File                                     | Role                                                              |
| ---------------------------------------- | ----------------------------------------------------------------- |
| `AbstractConfig.java`                    | Base config with `enabled`, `verbose`, and tunable helpers        |
| `AbstractSubsystemConfig.java`           | Extends `AbstractConfig` with subsystem-scoped tunable helpers    |
| `AbstractMotorSubsystemConfig.java`      | PID and feedforward gains shared by motor subsystems              |
| `AbstractSetAndSeekSubsystemConfig.java` | Trapezoidal profile constraints, setpoint limits, tolerances      |
| `AbstractVelocitySubsystemConfig.java`   | Max RPM, acceleration, idle RPM, and velocity tolerance           |
| `MotorConfig.java`                       | Motor CAN ID, gearing, current limits, and soft limits            |
| `FeedforwardConfig.java`                 | Feedforward gain bundle (kS, kV, kA)                              |
| `PidConfig.java`                         | PID gain bundle (kP, kI, kD)                                      |
| `SetAndSeekMotionConfig.java`            | Position motion profile parameters (velocity, acceleration, etc.) |
| `VelocityMotionConfig.java`              | Velocity motion profile parameters (max RPM, accel, tolerance)    |
| `SysIdRoutineConfig.java`                | SysId characterization timing and voltage parameters              |
| `ConfigurationLoader.java`               | Loads JSON configs from deploy and records to AdvantageKit        |
| `Pose2dDeserializer.java`                | Jackson deserializer for `Pose2d` (x, y meters; rotation degrees) |
| `RobotEnvironment.java`                  | Cached per-cycle DriverStation state (alliance, mode, FMS)        |
| `SubsystemsConfig.java`                  | Root config bundle with one field per subsystem config            |

## Adding a new config

1. Create a config class extending the appropriate abstract base (e.g.,
   `AbstractSetAndSeekSubsystemConfig` for a profiled mechanism).
2. Add public fields for mechanism-specific values with human-friendly default
   values.
3. Expose tunable getters using `readTunableNumber`, `readTunableDegrees`, etc.
4. Add the config as a public field in `SubsystemsConfig`.
5. Add a matching JSON block in `subsystems.json`, `subsystems-sim.json`, and
   `subsystems-test.json`.
