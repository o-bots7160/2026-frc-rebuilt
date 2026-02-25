# Glossary

A student-friendly reference for the robotics, programming, and game terms used
throughout this codebase. Definitions are short and practical — just enough to
get you moving. When a subsystem README uses a term from this glossary, it links
here so you can look it up without leaving the page.

---

## Acceleration

The rate at which speed changes over time. In our code, acceleration is
typically measured in degrees per second squared (deg/s²) or meters per second
squared (m/s²). Motion profiles cap acceleration to keep movements smooth and
protect mechanisms from sudden jolts.

## AdvantageKit

A logging and replay framework for FRC robots. It records every sensor reading,
motor output, and command state so you can replay an entire match on your laptop
after the fact. AdvantageKit also powers live-tunable values through
SmartDashboard so you can tweak gains without redeploying code.

## AprilTag

A black-and-white square marker with a unique ID printed on it. AprilTags are
placed at known locations on the FRC field. When a camera sees a tag, software
can calculate the camera's position and rotation relative to that tag, giving
the robot a field-position fix.

## CAN bus

Controller Area Network — a wiring standard that lets motor controllers,
sensors, and the roboRIO talk over a shared two-wire cable. Each device on the
bus has a unique numeric ID so messages reach the right place.

## Confidence

A score that tells us how much we trust a measurement. In vision, a high
confidence means the camera had a clear view of the target and the pose estimate
is likely accurate. Low confidence means the data is noisy or the target was
partially blocked.

## Deadband

A small range of input around zero that the code treats as exactly zero. It
filters out tiny joystick movements caused by stick drift or noise so the robot
does not creep when the driver's hands are off the controls.

## Encoder

A sensor that measures how far a shaft has turned. Motor controllers use
encoders to report position (rotations or radians) and velocity (rotations per
second). The turret, for example, reads its encoder to know the current heading.

## Feedforward

A control strategy that predicts how much motor effort a planned motion will
need before any error is measured. It works alongside [PID](#pid) — feedforward
handles the expected effort and PID corrects whatever is left over. A well-tuned
feedforward means the PID only has to make small corrections, resulting in
smoother motion.

### Feedforward gains

- **kS (static gain)** — the minimum voltage needed to overcome static friction
  and get the mechanism moving. Think of it as the "unstick" voltage.
- **kV (velocity gain)** — the voltage needed per unit of speed to keep the
  mechanism moving at a constant rate.
- **kA (acceleration gain)** — the voltage needed per unit of acceleration to
  change speed. Higher values mean the mechanism is heavier or has more inertia.

## Field-centric control

A drive mode where "forward" on the joystick always means toward the far end of
the field, regardless of which way the robot is currently facing. The robot uses
its [gyroscope](#gyroscope) heading to rotate the driver's input into the
correct chassis speeds.

## Flywheel

A spinning wheel (or pair of wheels) used to launch game pieces. The shooter
subsystem spins flywheels to a target RPM; when a game piece is pushed into
them, the stored rotational energy accelerates the piece outward.

## FUEL

The game piece for the 2026 FRC game REBUILT. FUEL is collected by the intake,
transported by the feeder, gated by the indexer, and launched by the shooter.

## Gear ratio

The ratio of motor rotations to mechanism rotations. A gear ratio of 47.5 : 1
means the motor turns 47.5 times for every single turn of the mechanism. Higher
ratios give more torque and precision but lower output speed. In our code, this
value is stored as `motorRotationsPerMechanismRotation`.

## Gyroscope

A sensor that measures how fast the robot is rotating (and by integration, what
heading it is currently facing). The drivebase uses the gyro for field-centric
control and odometry.

## Holonomic

A type of drive system that can move in any direction (forward, sideways,
diagonally) while independently rotating. Swerve drive is holonomic because each
module steers and drives independently.

## IO (Input/Output)

An abstraction layer in the AdvantageKit pattern. An IO interface defines what
data a subsystem reads from hardware (inputs) and what commands it sends
(outputs). A real implementation talks to actual sensors and motors; a
simulation implementation fakes the data so you can test without hardware.

## Kalman filter

A math technique that blends multiple noisy measurements into a single, more
accurate estimate. The drivebase's pose estimator uses a Kalman filter to
combine wheel [odometry](#odometry) (fast but drifts) with [vision](#apriltag)
fixes (slower but absolute) to produce a fused pose that is both smooth and
accurate.

## Latency

The time delay between when an event happens (a camera captures a frame) and
when the code processes it. Latency matters for aiming because the robot may
have moved since the camera took the picture. Latency compensation adjusts the
pose estimate backward in time to account for this delay.

## Odometry

A running estimate of the robot's position and heading based on wheel encoders
and the [gyroscope](#gyroscope). It updates every code cycle (usually 50 times
per second) and gives smooth, continuous tracking. The downside is that wheel
slip and measurement errors slowly cause the estimate to drift from reality.

## Omega

The symbol ω — rotational velocity around the vertical axis, measured in radians
per second. In chassis speed commands, omega controls how fast the robot spins.

## PID

Proportional-Integral-Derivative — a feedback controller that looks at the
difference between where you are and where you want to be (the **error**) and
calculates a correction. It is the most common control strategy in FRC.

### PID gains

- **P (proportional)** — output is proportional to the current error. Larger
  error → stronger push. This is usually the first gain you tune.
- **I (integral)** — output is proportional to the accumulated error over time.
  It slowly ramps up to fix a small steady-state error that P alone cannot
  eliminate. Use sparingly — too much causes oscillation.
- **D (derivative)** — output is proportional to how fast the error is changing.
  It acts as a brake, slowing the system as it approaches the target to reduce
  [overshoot](#overshoot). A small amount smooths the response.

## Pose

The combination of a robot's position on the field (X and Y in meters) and its
heading (rotation angle). Together, these three values fully describe where the
robot is and which direction it is facing.

## Profiled PID

A [PID](#pid) controller whose setpoint is not a single fixed target but a
smooth series of intermediate targets generated by a
[trapezoidal motion profile](#trapezoidal-motion-profile). This prevents the
controller from demanding full power at startup and produces controlled,
predictable motion.

## Overshoot

When a mechanism moves past its target before settling back. Overshoot happens
when the controller pushes too hard or does not brake soon enough. Tuning
[PID D gain](#pid-gains) or reducing [PID P gain](#pid-gains) can reduce it.

## Robot-centric control

A drive mode where "forward" on the joystick means forward relative to the
robot's current facing direction. If the robot is turned sideways, pushing
forward drives it sideways from the driver's perspective.

## RPM

Revolutions per minute — a measure of how fast a shaft or flywheel is spinning.
The shooter targets a specific RPM to launch FUEL at the correct speed.

## Set-and-seek

The control pattern used by mechanisms like the turret. "Set" a target position,
then "seek" it by stepping through a
[trapezoidal motion profile](#trapezoidal-motion-profile) each code cycle. The
base classes `AbstractSetAndSeekSubsystem` and `AbstractSetAndSeekCommand`
implement this pattern so every profiled mechanism works the same way. See the
[shared framework README](shared/README.md) for details.

## Soft limit

A software-enforced boundary that prevents a motor from driving past a
configured position. Unlike a physical limit switch, a soft limit uses the
encoder reading to stop the motor before the mechanism hits a hard stop.

## Swerve drive

A drivetrain where each wheel module can independently steer (rotate) and drive
(spin). This gives the robot [holonomic](#holonomic) motion — it can translate
in any direction while simultaneously rotating.

## SysId

System Identification — a WPILib tool that measures how a mechanism responds to
known voltages. It outputs [feedforward gains](#feedforward-gains) (kS, kV, kA)
that describe the mechanism's real-world behavior, which you plug into your
config so [feedforward](#feedforward) is accurate from the start.

## Theta

The robot's heading angle around the vertical axis, measured in radians. Zero
radians typically faces toward the far (red/blue) alliance wall.

## Transform

A description of how one reference frame is positioned and rotated relative to
another. For example, a camera transform describes where the camera sits on the
robot (forward/back, left/right, up/down) and which way it points. Transforms
let the code convert a camera-relative pose into a robot-relative or
field-relative pose.

## Trapezoidal motion profile

A motion plan that limits both speed and acceleration. It has three phases:
accelerate up to a cruise speed, hold that speed, then decelerate to a stop. The
velocity-over-time graph looks like a trapezoid. This keeps motion smooth,
prevents belt or chain shock, and protects the mechanism from sudden starts and
stops.

```
velocity
  ^            ____________  <- cruise at max velocity
  |           /            \
  |          /              \
  |         /                \
  |________/                  \________  -> time
           accel            decel
```

## Tunable

A configuration value that can be changed on the fly through the SmartDashboard
or AdvantageKit without redeploying code. Tunables let you adjust
[PID gains](#pid-gains), speed limits, and other settings while the robot is
running — invaluable during practice and tuning sessions.

## YAGSL

Yet Another Generic Swerve Library — a third-party library that manages swerve
drive kinematics, motor control loops, and odometry for FRC robots. In our
codebase, YAGSL handles the low-level module control inside the drivebase
subsystem so our code focuses on higher-level driving commands.
