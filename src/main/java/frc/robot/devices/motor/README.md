# Motor Abstraction

Provides a hardware-agnostic motor interface, abstract base classes for real and
simulated motors, and utility implementations for composite and disabled motors.
Every subsystem interacts with motors through the `Motor` interface so hardware
can be swapped, simulated, or disabled without changing subsystem code.

## Hierarchy

```
Motor (interface, extends MotorIO)
├── AbstractMotor (REV SparkMax base)
│   ├── AbstractSimMotor (SparkMaxSim + DCMotorSim)
│   │   └── AbstractVelocitySimMotor (velocity sim motor)
│   └── AbstractVelocityMotor (velocity real motor)
├── CompositeMotor (multi-motor adapter)
└── DisabledMotor (no-op fallback)
```

## How it works

- **`Motor`** defines the minimal control surface: set voltage, set speed, read
  position/velocity, stop, and reset the encoder. All subsystems depend only on
  this interface.
- **`MotorIO`** defines the telemetry surface. Its `@AutoLog` inner class
  `MotorIOInputs` holds position (radians, rotations, degrees), velocity (rad/s,
  RPM, raw encoder), voltage, current, and temperature. Every motor
  implementation populates these fields each loop so AdvantageKit can replay
  them.
- **`AbstractMotor`** wraps a REV SparkMax, handling configuration (current
  limits, inversion, soft limits, voltage compensation), unit conversions via
  gear ratios, and AdvantageKit input logging. It is the parent for both
  real-hardware and simulation motor classes.
- **`AbstractSimMotor`** extends `AbstractMotor` with a `SparkMaxSim` and
  `DCMotorSim` pair, letting the simulated SparkMax follow the same control
  modes and conversion factors as real hardware.
- **`AbstractVelocityMotor`** and **`AbstractVelocitySimMotor`** specialize
  their respective parents for velocity-controlled mechanisms (flywheels,
  rollers, belts), applying coast-mode idle and velocity-specific configuration.
- **`CompositeMotor`** wraps multiple motors behind a single `Motor` interface.
  One motor is the primary (providing encoder feedback); the rest receive the
  same commands as followers. Null or disabled followers are filtered out
  automatically.
- **`DisabledMotor`** is a no-op implementation used when a subsystem is
  disabled or has no physical hardware. Commands are silently ignored and
  getters return neutral values.

## Code structure

| File                            | Role                                                        |
| ------------------------------- | ----------------------------------------------------------- |
| `Motor.java`                    | Interface defining the motor control contract               |
| `MotorIO.java`                  | Telemetry interface with `@AutoLog` input container         |
| `AbstractMotor.java`            | SparkMax base wrapper with config, gearing, and soft limits |
| `AbstractSimMotor.java`         | Simulation motor base with `SparkMaxSim` + `DCMotorSim`     |
| `AbstractVelocityMotor.java`    | Velocity-mode real motor (coast idle, velocity encoding)    |
| `AbstractVelocitySimMotor.java` | Velocity-mode simulation motor                              |
| `CompositeMotor.java`           | Multi-motor adapter (primary + followers)                   |
| `DisabledMotor.java`            | No-op motor for disabled subsystems                         |

## Adding a new motor type

1. **Position-controlled** — extend `AbstractMotor` (real) and
   `AbstractSimMotor` (sim). Override `configureMotor()` to set brake mode,
   current limits, and conversion factors.
2. **Velocity-controlled** — extend `AbstractVelocityMotor` (real) and
   `AbstractVelocitySimMotor` (sim). The base classes already apply coast mode
   and velocity encoding.
3. Follow the turret and shooter as concrete examples.
