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

## Positioning and units (student guide)

The turret API uses degrees so it is easy to think about angles. Internally we
convert to radians because WPILib math uses radians. The conversion happens once
when a target is set, and once when we log values.

- Commands call `setTarget()` with degrees.
- The base class converts degrees to radians for the controller.
- The motor wrapper configures the encoder so it already reports radians.
- Logs are written in both radians and degrees for clarity.

## Gear setup and ratio math

The Spark MAX is using the **internal relative encoder**, which is attached to
the motor shaft. That means the encoder measures motor rotations, not turret
rotations. We must scale that sensor by the full gear reduction from the motor
to the turret.

We store the ratio as **motor rotations per mechanism rotation**. In simple
terms:

```
motor rotations per turret rotation = gearbox ratio × (driven teeth / driver teeth)
```

Example for a 5:1 gearbox, 16T driving 152T:

```
ratio = 5 × (152 / 16) = 47.5
```

That ratio is set in `subsystems.json` as
`turretMotorConfig.motorRotationsPerMechanismRotation`. The motor wrapper then
converts it to **radians per motor rotation** so the Spark MAX encoder reports
true turret radians.

If the turret rotates twice as far as commanded, the ratio is usually off by a
factor of two. That typically means one gear stage or tooth count is different
from what we modeled. Always verify the real gearbox ratio and sprocket tooth
counts.

## Motion profiling and PID control

The turret uses a **profiled PID controller** with a trapezoidal motion profile.
This gives smooth movement that respects maximum speed and acceleration limits.

Key ideas:

- **Profile constraints**: `maximumVelocityDegreesPerSecond` and
  `maximumAccelerationDegreesPerSecondSquared` limit how fast we move.
- **Profiled PID**: `kP`, `kI`, `kD` control how aggressively we track the
  profile.
- **Feedforward**: `kS`, `kV`, `kA` add expected motor voltage for velocity and
  acceleration so PID does less work.

All of these are read as tunable values so you can change them live in
AdvantageKit without redeploying.

### Trapezoidal profile (ASCII sketch)

Velocity vs. time for a trapezoidal move:

```
velocity
  ^            ____________  <- cruise at max velocity
  |           /            \
  |          /              \
  |         /                \
  |________/                  \________  -> time
           accel            decel
```

Position vs. time for the same move (smooth S-curve):

```
position
  ^         _____________
  |        /             \
  |      _/               \_
  |    _/                   \_
  |___/                       \____  -> time
```

### How a move works, step-by-step

1. A command sets a target in degrees.
2. The base class clamps the target to safety limits.
3. The trapezoid profile generates a smooth setpoint each loop.
4. PID + feedforward compute the voltage needed to reach that setpoint.
5. The motor wrapper applies that voltage and logs the results.

### Tuning checklist

- Start with correct gear ratio and limits.
- Increase `kP` until it tracks well but does not oscillate.
- Add a small `kD` to reduce overshoot.
- Use `kS`, `kV`, `kA` from SysId to reduce steady error.
- Keep the profile speed/accel low enough to avoid twisting belts or chains.

## Where to find the settings

- Turret config values live in `src/main/deploy/subsystems.json` under
  `turretSubsystem`.
- Motor config values live in the same file under
  `turretSubsystem.turretMotorConfig`.
- The base behavior lives in `AbstractSetAndSeekSubsystem` and `AbstractMotor`.

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
