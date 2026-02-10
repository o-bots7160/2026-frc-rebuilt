# Subsystems

Subsystems are the building blocks of a WPILib command-based robot. Each
subsystem owns a piece of hardware (a drivetrain, a turret, a shooter) and
exposes a safe public API that commands use to make the hardware do things. The
command scheduler guarantees that only one command controls a subsystem at a
time, which prevents two pieces of code from fighting over the same motor.

This folder contains one subfolder per mechanism on the robot. Each folder holds
the subsystem class, its commands, its configuration, and the hardware
abstractions it needs.

## Folder layout

Every mechanism folder follows the same structure. Not every mechanism needs
every subfolder — a simple roller intake may not need a `devices/` folder if the
base IO layer is sufficient.

```
subsystems/<mechanism>/
├── <Mechanism>Subsystem.java        Main subsystem class
├── README.md                        Documentation for this mechanism
├── commands/
│   ├── <Action>Command.java         One class per command
│   └── <Mechanism>SubsystemCommandFactory.java
├── config/
│   ├── <Mechanism>SubsystemConfig.java
│   └── <Mechanism>MotorConfig.java  (if motors need their own settings)
├── devices/                         Concrete motor/sensor wrappers (optional)
│   ├── <Mechanism>Motor.java        Real hardware implementation
│   └── <Mechanism>SimMotor.java     Simulation implementation
└── io/                              AdvantageKit IO layer (optional)
    ├── <Mechanism>IO.java           IO interface + logged inputs
    └── <Mechanism>IO<Impl>.java     Hardware or sim implementation
```

### `devices/` vs. `io/` — what goes where?

These two folders serve different purposes and a subsystem may have one, both,
or neither:

- **`devices/`** contains concrete motor and sensor wrapper classes. These
  extend the shared motor abstractions (see the
  [shared framework README](../shared/README.md)) and handle device-specific
  setup like encoder scaling, current limits, and inversion. The turret uses
  this pattern because `AbstractSetAndSeekSubsystem` already defines the IO
  contract through its motor interface.

- **`io/`** contains [AdvantageKit IO](../GLOSSARY.md#io-inputoutput) interfaces
  and their implementations. The IO interface declares what data the subsystem
  reads (inputs) and what it writes (outputs). Separate real and sim
  implementations let you run the exact same subsystem logic with or without
  hardware. The drivebase and vision subsystems use this pattern because their
  data flow is more complex than a single motor.

A mechanism that uses `AbstractSetAndSeekSubsystem` typically only needs
`devices/` because the base class handles IO through the motor interface. A
mechanism with custom sensor pipelines or third-party library integration (like
YAGSL or PhotonVision) typically uses `io/`.

## Inheritance hierarchy

All subsystems, commands, configs, and factories extend shared base classes in
the [`shared/`](../shared/README.md) package. The hierarchy keeps common
behavior in one place and lets each mechanism focus on what makes it unique.

```
WPILib SubsystemBase
 └─ AbstractSubsystem<TConfig>           shared/subsystems/
     └─ AbstractSetAndSeekSubsystem      shared/subsystems/
         └─ TurretSubsystem, etc.        subsystems/<mechanism>/

WPILib Command
 └─ AbstractSubsystemCommand             shared/commands/
     └─ AbstractSetAndSeekCommand        shared/commands/
         └─ MoveTurretToAngleCommand     subsystems/<mechanism>/commands/

AbstractSubsystemCommandFactory          shared/commands/
 └─ AbstractSetAndSeekCommandFactory     shared/commands/
     └─ TurretSubsystemCommandFactory   subsystems/<mechanism>/commands/

AbstractConfig                           shared/config/
 ├─ AbstractSetAndSeekSubsystemConfig    shared/config/
 │   └─ TurretSubsystemConfig           subsystems/<mechanism>/config/
 └─ AbstractMotorConfig                  shared/config/
     └─ TurretMotorConfig               subsystems/<mechanism>/config/
```

See the [shared framework README](../shared/README.md) for details on what each
base class provides.

## Mechanism index

Each mechanism has its own README with an overview, how-it-works guide,
configuration reference, and code-structure table. Click through for details.

| Mechanism                                            | Description                                                  | Status      |
| ---------------------------------------------------- | ------------------------------------------------------------ | ----------- |
| [AprilTag Vision](apriltagvision/README.md)          | Estimates robot pose from AprilTag camera data               | Implemented |
| [Climber](climber/README.md)                         | Raises the robot onto tower rungs for endgame points         | Planned     |
| [Drivebase](drivebase/README.md)                     | Swerve drivetrain for field movement and autonomous paths    | Implemented |
| [Driver Camera Vision](drivercameravision/README.md) | Provides a driver-facing camera feed for operator visibility | Implemented |
| [Feeder](feeder/README.md)                           | Centers FUEL from the hopper sides into the indexer lane     | Planned     |
| [Hopper](hopper/README.md)                           | Stores and stages collected FUEL for the feeder              | Planned     |
| [Indexer](indexer/README.md)                         | Gates FUEL release into the shooter for timed shots          | Planned     |
| [Intake](intake/README.md)                           | Collects FUEL from the field and passes it to the hopper     | Planned     |
| [Robot State](robotstate/README.md)                  | Single authoritative source of robot pose on the field       | Implemented |
| [Shooter](shooter/README.md)                         | Spins flywheels to launch FUEL at scoring targets            | Planned     |
| [Turret](turret/README.md)                           | Single-axis rotating mount for aiming the shooter            | Implemented |

## Adding a new subsystem

1. Create a new folder under `subsystems/` named after the mechanism (lowercase,
   no spaces).
2. Add a `README.md` following the standard template (see any implemented
   subsystem for the section order: Overview, How it works, Configuration, Code
   structure, Status/TODO).
3. Create the subsystem class extending `AbstractSubsystem` (or
   `AbstractSetAndSeekSubsystem` for profiled mechanisms).
4. Add a config class extending `AbstractConfig` (or
   `AbstractSetAndSeekSubsystemConfig`) and register it in
   `SubsystemsConfig.java`.
5. Add the subsystem's entry to `subsystems.json`, `subsystems-sim.json`, and
   `subsystems-test.json` in the `deploy/` folder.
6. Create a `commands/` folder with at least a command factory class.
7. Instantiate the subsystem in `RobotContainer` in the correct dependency order
   (see the [copilot instructions](../../../.github/copilot-instructions.md) for
   wiring rules).
