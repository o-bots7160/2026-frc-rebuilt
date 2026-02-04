# Turret subsystem

## Role (game/mechanical)

The turret is a single-axis rotating mount. It turns left and right so other
mechanisms can aim without moving the entire robot. The turret uses one Spark
MAX motor controller and a gear reduction so it can move smoothly and hold a
stable heading.

## Control and configuration (programming)

- `TurretSubsystem` extends `AbstractSetAndSeekSubsystem`, which provides the
  trapezoidal motion profile and the set-and-seek API.
- `TurretSubsystemConfig` holds the profile limits and control gains. The config
  values are read through AdvantageKit tunables so they can be adjusted while
  the robot is running.
- `TurretMotorConfig` holds motor-specific details such as CAN ID, inversion,
  current limits, and gear ratio.
- When running in simulation, the subsystem uses `TurretSimMotor` (which extends
  `AbstractSimMotor`) so the motion profile behaves the same without hardware.

## Positioning and units (student guide)

We use degrees in the public API because it is easier for humans to read and
think about. WPILib math uses radians, so we convert once when a target is set.

- Commands call `setTarget(double)` with a target in degrees.
- `AbstractSetAndSeekSubsystem` clamps the target between the configured minimum
  and maximum and then converts to radians for the controller.
- `TurretMotor` configures the Spark MAX encoder conversion factors so the
  encoder reports turret radians, not raw motor rotations.
- AdvantageKit logs are recorded in degrees in the base class for clarity.

## Gear setup and ratio math

The Spark MAX uses the **internal relative encoder**, which measures motor
rotations. The turret itself moves much slower because of the gear reduction. We
must scale the encoder using the full gear ratio.

We store the ratio as **motor rotations per turret rotation**:

motor rotations per turret rotation = gearbox ratio × (driven teeth / driver
teeth)

Example: for a 5:1 gearbox with a 16-tooth gear driving a 152-tooth gear:

ratio = 5 × (152 / 16) = 47.5

That ratio is stored in `subsystems.json` as
`turretSubsystem.turretMotorConfig.motorRotationsPerMechanismRotation`. The
motor wrapper converts it to **radians per motor rotation** so the encoder
reports true turret radians.

If the turret moves twice as far as expected, the gear ratio is usually off by a
factor of two. Check every gear stage and tooth count to confirm the real ratio.

## Motion profiling and PID control

The turret uses a **profiled PID controller** with a trapezoidal motion profile.
This creates smooth movement that respects speed and acceleration limits.

Key ideas:

- **Profile limits**: `maximumVelocityDegreesPerSecond` and
  `maximumAccelerationDegreesPerSecondSquared` cap the requested motion.
- **PID gains**: `kP`, `kI`, `kD` correct error between the setpoint and the
  measured position.
- **Feedforward gains**: `kS`, `kV`, `kA` estimate the voltage needed for the
  planned motion so PID only has to correct the leftover error.

### Terminology deep dive (student friendly)

- **Profile limits (velocity/acceleration limits)**: These are safety and
  smoothness guards. The profile planner is not allowed to request speeds or
  accelerations above these limits. If the target is far away, the motion ramps
  up to the max speed, cruises, then ramps down. If the target is close, it
  ramps up and down without ever reaching full speed.
- **Trapezoidal profile**: A plan for how fast we should move over time. It uses
  three phases: accelerate, cruise, and decelerate. The speed plot looks like a
  trapezoid, which is why it has that name. This keeps motion smooth and
  prevents belt or chain shock.
- **Profiled PID**: PID is a feedback controller that compares where we are to
  where we want to be. “Profiled” means the target is not a single jump; it is
  the smooth moving setpoint from the trapezoid profile. PID reacts to error in
  real time, so it can correct for friction, battery sag, or small bumps in the
  mechanism.
  - **`P` (proportional)**: Output is proportional to the current error. It
    pushes harder when the error is large.
  - **`I` (integral)**: Output is proportional to the accumulated error over
    time. It fixes small leftover error by adding up error over time.
  - **`D` (derivative)**: Output is proportional to the rate of change of the
    error. It slows the system as it approaches the target to reduce overshoot.
- **Feedforward**: A best-guess motor voltage based on the planned motion. This
  handles the expected effort so PID only has to correct the difference.
  Feedforward is open-loop, so it does not look at error; it just predicts how
  much voltage the motion will need. When feedforward is close, the PID
  corrections stay small and the turret feels smoother.
  - **`kS` (static)**: Extra voltage to overcome static friction (the “stiction”
    bump).
  - **`kV` (velocity)**: Voltage per unit speed (how much it takes to keep
    moving).
  - **`kA` (acceleration)**: Voltage per unit acceleration (how much it takes to
    speed up).

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

Position vs. time for the same move:

```
position
  ^         _____________
  |        /             \
  |      _/               \_
  |    _/                   \_
  |___/                       \____  -> time
```

### How a move works, step by step

1. A command sets a goal with `setTarget(double)` in degrees.
2. The base class clamps the target to safety limits.
3. The trapezoid profile produces a smooth setpoint each loop.
4. PID + feedforward compute the voltage needed to track the setpoint.
5. The motor wrapper applies the voltage and the base class logs telemetry.

### How commands drive the turret

- `MoveTurretToAngleCommand` extends `AbstractSetAndSeekCommand`.
- On initialize, the command calls `setTarget(double)` once using the supplier.
- During execute, the command calls `seekTarget()` every loop to advance the
  profile.
- The command finishes when `isProfileSettled()` returns true.
- If the command is interrupted, the base command schedules a settle command so
  the turret decelerates safely.

## Tuning checklist (student friendly)

- Confirm the gear ratio first.
- Choose safe max speed and acceleration values before tuning gains.
- Use SysId to get `kS`, `kV`, and `kA` so feedforward is close.
- Increase `kP` until tracking is strong but not oscillating.
- Add a small `kD` if you see overshoot.
- Keep test moves simple and repeatable so changes are easy to compare.

## Where to find the settings

- Turret configuration lives in `src/main/deploy/subsystems.json` under
  `turretSubsystem`.
- Motor configuration lives in the same file under
  `turretSubsystem.turretMotorConfig`.
- Shared base behavior lives in `AbstractSetAndSeekSubsystem` and
  `AbstractMotor`.

## Code structure and maintenance

- `TurretSubsystem` builds either `TurretMotor` (real hardware) or
  `TurretSimMotor` (simulation).
- `TurretMotor` extends `AbstractMotor` and applies inversion, current limits,
  and encoder scaling.
- `MoveTurretToAngleCommand` is the main profiled move command.
- `TurretSubsystemCommandFactory` builds commands so `RobotContainer` only wires
  them.
- Keep public APIs in degrees, keep command names ending in `Command`, and keep
  command factories inside `commands/`.

## TODO

- Revisit aiming presets and vision integration once shooter testing begins.
