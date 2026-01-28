# Turret subsystem

## Role (game/mechanical)

Spins and aims a single-axis turret driven by one SparkMax. Uses hard motion
limits and trapezoidal profiles so we can track the scoring hub or pickup zones
without slamming end stops. Holds heading during volleys so the indexer can feed
at a steady cadence.

## Control and configuration (programming)

- Public API accepts angles in degrees; conversions happen inside the motor
  wrapper and set-and-seek base.
- `TurretSubsystemConfig` supplies tunable limits, max velocity/accel, gear
  ratio, CAN ID, inversion, and current limits; values are read through
  AdvantageKit tunables.
- Uses `AbstractSetAndSeekSubsystem` to run trapezoidal profiles and enforce
  min/max angle guards before applying motor output.
- In sim, swaps to `AbstractSimMotor` with the same limits, gearing, and profile
  bounds so autonomous and command logic behave consistently.
- Works with vision to auto-aim and should align with shooter RPM readiness
  before feeding pieces.

## Code structure and maintenance

- Classes:
  - `TurretSubsystem` extends `AbstractSetAndSeekSubsystem` and owns motor
    creation (SparkMax vs. `AbstractSimMotor`).
  - `devices/TurretMotor` wraps the SparkMax, handles inversion/current limits,
    and scales encoder units to radians while reporting degrees to callers.
  - `commands/MoveTurretToAngleCommand` is the profiled move command; the
    `commands/TurretSubsystemCommandFactory` builds supplier-based and fixed
    targets to keep `RobotContainer` wiring clean.
  - `config/TurretSubsystemConfig` is loaded from `subsystems.json` and exposes
    tunable suppliers for limits, gear ratio, current limit, inversion, and CAN
    ID.
- Reviewer notes: keep public APIs in degrees; preserve AdvantageKit logging of
  setpoints/positions via the base class; keep command names ending in `Command`
  and factories in `commands/`.

## TODO

- Revisit aiming presets and vision integration once shooter and field testing
  start.
