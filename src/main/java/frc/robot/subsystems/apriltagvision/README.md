# AprilTag Vision subsystem

## Overview

The AprilTag Vision subsystem gives the robot field awareness. It uses cameras
to detect [AprilTags](../../GLOSSARY.md#apriltag) — black-and-white square
markers placed at known locations on the FRC field — and computes
[pose](../../GLOSSARY.md#pose) estimates that tell the robot where it is. These
estimates are forwarded to the [Robot Pose subsystem](../robotpose/README.md),
which fuses them with [odometry](../../GLOSSARY.md#odometry) to keep the robot's
field position accurate for aiming, shooting, and autonomous paths.

## How it works

### From camera frame to field position

1. A camera captures a frame and PhotonVision (a vision processing library)
   detects any [AprilTags](../../GLOSSARY.md#apriltag) in the image.
2. For each tag, PhotonVision solves for the camera's position and rotation
   relative to the tag. If multiple tags are visible, it uses all of them
   together for a more accurate multi-tag estimate.
3. The [IO](../../GLOSSARY.md#io-inputoutput) layer
   (`AprilTagVisionIOPhotonVision`) converts the camera-relative observation
   into a robot [pose](../../GLOSSARY.md#pose) using the known camera
   [transform](../../GLOSSARY.md#transform) — the physical offset and angle of
   the camera on the robot.
4. `AprilTagPoseEstimator` filters the observation: it rejects poses that fall
   outside the field boundaries, rejects single-tag observations that are too
   ambiguous, and calculates a [confidence](../../GLOSSARY.md#confidence) value
   (standard deviations) based on tag distance and count.
5. When wheel slip moves odometry far from the field position, the estimator
   accepts a correction only after several consecutive multi-tag poses agree
   with each other. Single-tag poses cannot activate this recovery path.
6. Accepted measurements are forwarded to the
   [Robot Pose subsystem](../robotpose/README.md) through a
   `VisionMeasurementConsumer` callback wired in `RobotContainer`.

### Why confidence matters

Farther tags and fewer tags mean less certainty. The estimator scales the
standard deviations by `(averageTagDistance²) / tagCount`, so a tag 3 meters
away with one tag visible produces 9× the uncertainty of a tag 1 meter away. The
[Kalman filter](../../GLOSSARY.md#kalman-filter) in the drivebase uses these
values to decide how much to trust each vision fix.

### Simulation

In simulation, `AprilTagVisionIOPhotonVisionSim` creates a simulated camera that
renders synthetic tag observations based on the robot's current
[odometry](../../GLOSSARY.md#odometry) pose. This lets you test vision-dependent
autonomous paths and aiming without physical cameras.

## Configuration

Settings live in `subsystems.json` under `aprilTagVisionSubsystem`. All values
are [tunable](../../GLOSSARY.md#tunable).

| Setting                            | Units         | Purpose                                                                                                                           |
| ---------------------------------- | ------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| `cameras`                          | —             | Map of camera names to robot-to-camera [transforms](../../GLOSSARY.md#transform) (x, y, z in meters; roll, pitch, yaw in radians) |
| `maximumAmbiguity`                 | dimensionless | Rejection threshold for single-tag observations                                                                                   |
| `linearStandardDeviationBaseline`  | meters        | Base uncertainty for x/y at 1 m with 1 tag                                                                                        |
| `angularStandardDeviationBaseline` | radians       | Base uncertainty for rotation at 1 m with 1 tag                                                                                   |
| `recoveryMinimumTagCount`           | count         | Minimum tags required to recover from a large odometry error                                                                      |
| `recoveryRequiredConsecutiveFrames` | frames        | Consistent multi-tag poses required before recovery                                                                               |
| `recoveryMaximumFrameTranslationMeters` | meters    | Maximum translation difference between consecutive recovery poses                                                                |
| `recoveryMaximumFrameRotationDegrees` | degrees     | Maximum heading difference between consecutive recovery poses                                                                    |

### Camera transform tips

- Measure from the robot center to each camera lens in meters.
- X is forward, Y is left, Z is up (WPILib convention).
- Roll, pitch, yaw are in radians.
- Getting the transform wrong shifts every vision pose by the error amount — if
  aiming is consistently off by the same amount, check the camera transform
  first.

## Code structure

| File                                        | Purpose                                                                                        |
| ------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| `AprilTagVisionSubsystem.java`              | Main subsystem — owns camera IO instances, processes observations, and forwards accepted poses |
| `AprilTagPoseEstimator.java`                | Filtering and confidence logic — testable without hardware or subsystem infrastructure         |
| `config/AprilTagVisionSubsystemConfig.java` | Tunables for cameras, ambiguity threshold, and standard deviation baselines                    |
| `io/AprilTagVisionIO.java`                  | AdvantageKit IO interface defining logged inputs (connection status, observations, tag IDs)    |
| `io/AprilTagVisionIOPhotonVision.java`      | Real hardware implementation — talks to PhotonVision cameras                                   |
| `io/AprilTagVisionIOPhotonVisionSim.java`   | Simulation implementation — renders synthetic tag observations from odometry                   |

## Status / TODO

### Done

- Multi-camera AprilTag pose estimation with PhotonVision.
- Observation filtering (ambiguity, field bounds, tag count).
- Confidence-weighted standard deviations for Kalman filter fusion.
- Multi-frame, multi-tag recovery from wheel-slip odometry drift.
- Simulation support with synthetic tag observations.
- Per-camera and summary AdvantageKit logging.
- Disconnected-camera alerts on the Driver Station.

### TODO

- Run latency compensation so turret and shooter aim use current data.
- Provide a toggle for driver-assist aiming vs. pure manual control.
- Expose ready/valid signals so commands only act on good frames.
