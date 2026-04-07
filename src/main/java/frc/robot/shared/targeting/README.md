# Targeting

Provides shoot-on-the-move compensation and ball-flight visualization so the
turret can aim accurately while the robot is driving.

## How it works

### Shoot-on-the-move compensation

When the robot is moving, a launched ball inherits the robot's velocity and
drifts during its time of flight. `ShootOnTheMoveCalculator` solves for a
compensated aim point — a "virtual" target the turret should point at so the
ball arrives at the real target after accounting for that drift. The solver uses
Newton's method to converge on the correct aim point in 1–3 iterations.

### Ball-flight simulation

`BallFlightSimulator` is a simulation-only utility that models launched Fuel as
3D projectiles. Each cycle it checks whether the shooter and indexer are ready
to fire. When they are, a new ball is spawned at the turret exit with a velocity
derived from the current flywheel RPM. All active balls advance through simple
projectile kinematics (gravity and drag) and their positions are logged as a
`Pose3d[]` for AdvantageScope's 3D field view.

This class is not a subsystem and does not participate in the command scheduler.
It is instantiated in `RobotContainer` only when
`RobotEnvironment.isSimulation()` is true, and its `periodic()` method is called
from the container's periodic loop.

## Code structure

| File                            | Role                                                           |
| ------------------------------- | -------------------------------------------------------------- |
| `ShootOnTheMoveCalculator.java` | Compensated aim-point solver for turret aiming while driving   |
| `BallFlightSimulator.java`      | Simulation-only 3D projectile visualization for AdvantageScope |
