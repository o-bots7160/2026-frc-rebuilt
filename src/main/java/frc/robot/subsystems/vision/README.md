# Vision subsystem

## Role (game/mechanical)

Vision is used in a few different ways, with AprilTag processing as the first
priority. `AprilTagVisionSubsystem` gives the robot field awareness. It uses
AprilTags to estimate pose on the field so we can accurately control the turret
for aiming and shooting.

`DriverCameraSubsystem` provides a field view from the robot perspective. The
robot perspective may be necessary from certain driver stations for navigating
out of jams or to find FUEL.

Stretch goals include a birds-eye view of the robot and object detection to
locate FUEL. Switching between the driver camera and birds-eye view would be
ideal for navigating and FUEL collection when the driver cannot see the robot.
The birds-eye view would likely be better to help the driver determine which way
to turn to collect cells. Similarly, object detection can help guide the driver
to the largest deposit of FUEL.

## How vision works (student guide)

Vision is like the robot’s eyes. The camera sees AprilTags on the field and
figures out where they are in the image. Because the size and layout of each tag
is known, the software can solve for the camera’s position and rotation relative
to the tag. That gives us a **pose estimate** (position + heading) for the
camera.

The robot already has its own estimate from drive sensors. Wheel encoders tell
us how far we rolled and the gyro tells us our heading. That is called
**odometry**. Odometry is fast and smooth, but it slowly drifts because wheels
can slip.

Vision helps correct that drift. When a tag is seen, the vision system reports
the pose estimate and a confidence value. The robot state subsystem fuses the
two sources: it trusts odometry for short-term motion and uses the vision pose
to nudge the estimate back to the correct spot. That combination keeps the
robot’s field position accurate so the turret, shooter, and autonomous paths
stay aligned.

## Control and configuration (programming)

### Done

- Estimates robot pose relative to the field using tag observations.
- Tracks goals or pickup spots and shares angles to the turret and drivebase.
- Flags target confidence so commands can decide when to trust vision over
  odometry.
- Publishes pose and target info to AdvantageKit; robot state can fuse it with
  odometry.
- Document camera mounting offsets in config files to keep transforms correct.

### TODO

- Run latency compensation so turret and shooter aim use current data.
- Provide a toggle for driver-assist aiming vs. pure manual control.
- In sim, feed synthetic tag data so autonomous paths can be tested without
  cameras.
- Expose ready/valid signals so commands only act on good frames.

## Subsystem file structure and responsibilities

- `AprilTagVisionSubsystem`: Main subsystem that owns the AprilTag pipeline and
  publishes pose estimates and targets for other subsystems.
- `DriverCameraSubsystem`: Manages the driver-facing camera feed for operator
  visibility.
- `AprilTagPoseEstimator`: Converts AprilTag detections into pose estimates and
  applies camera transforms.
- Pose measurement consumer: a callback passed to the vision subsystem to
  forward accepted poses to robot state or other systems.
- `config/AprilTagVisionSubsystemConfig`: Tunables and mounting offsets for
  AprilTag vision.
- `config/DriverCameraSubsystemConfig`: Tunables for the driver camera setup.
- `io/AprilTagVisionIO`: Interface for camera IO and vision data inputs.
- `io/AprilTagVisionIOPhotonVision`: PhotonVision hardware implementation.
- `io/AprilTagVisionIOPhotonVisionSim`: Simulation implementation for testing
  without hardware.

## Code structure and maintenance

- Classes (planned): vision subsystem, pipeline/camera IO, and adapters for
  turret/drivebase consumers.
- Reviewer notes: keep transforms in config, not code; gate outputs with
  confidence to avoid bad frames driving commands.

## Glossary (student friendly)

- **AprilTag**: A black-and-white square marker with a known ID that helps the
  robot measure its location.
- **Pose**: The robot’s position on the field plus its heading (direction it is
  facing).
- **Odometry**: A running estimate of pose based on wheel and gyro sensors. It
  is smooth but can drift over time.
- **Transform**: A description of how one reference frame is positioned and
  rotated relative to another (for example, camera to robot).
- **Latency**: A time delay between when the camera saw a tag and when the data
  reaches the robot code.
- **Confidence**: A score that tells us how much we trust a vision estimate.

## TODO

- Select camera hardware and finalize AprilTag pipeline; add calibration steps
  and config fields once hardware is mounted.
- Add target config so turret can be aimed at hub versus targets.
