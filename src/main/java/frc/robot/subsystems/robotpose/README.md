# Robot Pose subsystem

## Overview

The Robot Pose subsystem is the single source of truth for the robot's position
on the field. Every command or subsystem that needs to know "where is the robot
right now?" reads its answer from here.

This subsystem does not own hardware. Instead, it collects pose data from two
sources and exposes the best-available estimate:

1. **Odometry** — wheel [encoder](../../GLOSSARY.md#encoder) and
   [gyroscope](../../GLOSSARY.md#gyroscope) readings fused inside the
   drivebase's YAGSL [pose estimator](../../GLOSSARY.md#pose-estimator).
2. **Vision** — camera-based measurements from the
   [AprilTag vision](../apriltagvision/README.md) subsystem, forwarded to the
   same YAGSL estimator for [Kalman-filter](../../GLOSSARY.md#kalman-filter)
   fusion with latency compensation.

The result is a continuously updated `Pose2d` (x, y in meters; heading in
radians) that commands use for field-relative aiming, path following, and
autonomous decisions.

## How it works

### Data flow

```
AprilTagVision ─► addVisionMeasurement() ─► YAGSL PoseEstimator (in drivebase)
                                                      │
                                                      ▼
                                            fusedPoseSupplier ─► RobotPoseSubsystem
                                                                       │
DriveBase (odometry) ──► odometryOnlyPoseSupplier ─────────────────────┘
```

1. The drivebase runs [odometry](../../GLOSSARY.md#odometry) each cycle,
   producing a raw wheel+gyro pose.
2. Vision subsystems call `addVisionMeasurement()` with a timestamped pose and
   uncertainty matrix. The measurement is forwarded to the drivebase's YAGSL
   `SwerveDrivePoseEstimator`.
3. YAGSL fuses odometry and vision using a
   [Kalman filter](../../GLOSSARY.md#kalman-filter) with latency compensation
   and uncertainty weighting.
4. Each cycle, `RobotPoseSubsystem.periodic()` reads the fused result from the
   drivebase and stores it as the authoritative pose.

### Vision fusion toggle

The `enableVisionFusion` config flag lets operators disable vision input without
redeploying. When disabled:

- Vision measurements are still recorded in AdvantageKit logs for replay
  analysis.
- Measurements are **not** forwarded to the pose estimator, so the estimate
  relies on odometry alone.

This is useful when cameras are disconnected, producing bad data, or when you
want to evaluate pure odometry drift.

### Pose reset

`resetPose(Pose2d)` resets both the local estimate and the drivebase's internal
estimator. Use this at the start of autonomous to seed the robot's position from
a known starting pose.

`resetPoseFromVision()` resets the pose to the most recent vision measurement.
Call this before autonomous starts so the estimator begins at a camera-derived
position rather than the origin. If no vision measurement has been received, the
reset is skipped and a warning is logged.

### Distance helper

`getDistanceToPointMeters(Translation2d)` computes the straight-line distance
from the robot's current estimated position to a field-relative target. Commands
use this for distance-based calculations like shooter RPM scaling or approach
detection.

## Subsystem decoupling

`RobotPoseSubsystem` never holds a direct reference to another subsystem. All
dependencies are injected as suppliers and consumers in the constructor:

| Parameter                  | Type                        | Source                          |
| -------------------------- | --------------------------- | ------------------------------- |
| `fusedPoseSupplier`        | `Supplier<Pose2d>`          | DriveBase fused pose getter     |
| `odometryOnlyPoseSupplier` | `Supplier<Pose2d>`          | DriveBase raw odometry getter   |
| `visionForwarder`          | `VisionMeasurementConsumer` | DriveBase vision-add method     |
| `odometryResetConsumer`    | `Consumer<Pose2d>`          | DriveBase odometry reset method |

Wiring happens in `RobotContainer`, keeping both subsystems independent.

## Configuration

Settings live in `src/main/deploy/subsystems.json` (or sim/test variants) under
the `robotPoseSubsystem` key.

### Key tunables

| Setting              | Type    | Default | Purpose                                     |
| -------------------- | ------- | ------- | ------------------------------------------- |
| `enableVisionFusion` | boolean | true    | Allow vision measurements into the estimate |

## Telemetry

The subsystem publishes the following values each cycle:

| SmartDashboard key                           | Description                                 |
| -------------------------------------------- | ------------------------------------------- |
| `RobotPoseSubsystem/EstimatedXMeters`        | Fused X position in meters                  |
| `RobotPoseSubsystem/EstimatedYMeters`        | Fused Y position in meters                  |
| `RobotPoseSubsystem/EstimatedHeadingDegrees` | Fused heading in degrees                    |
| `RobotPoseSubsystem/HasVisionMeasurement`    | True after first vision measurement arrives |
| `RobotPoseSubsystem/EnableVisionFusion`      | Current state of the vision fusion toggle   |
| `RobotPoseSubsystem/Field`                   | Field2d widget for dashboard visualization  |

AdvantageKit logs the full `RobotPoseIOInputs` structure (estimated pose,
odometry-only pose, last vision pose, timestamps) under the `RobotPose` key for
replay analysis.

## Code structure

| File                                   | Purpose                                                             |
| -------------------------------------- | ------------------------------------------------------------------- |
| `RobotPoseSubsystem.java`              | Reads fused pose, accepts vision measurements, exposes pose getters |
| `config/RobotPoseSubsystemConfig.java` | Tunable config for vision fusion gating                             |
| `io/RobotPoseIO.java`                  | AdvantageKit IO interface defining telemetry fields                 |

## Status / TODO

### Done

- Centralized pose authority with Kalman-filter-based vision fusion.
- Runtime-tunable vision fusion toggle.
- Pose reset from autonomous starting position or latest vision measurement.
- Distance-to-target helper for downstream commands.
- Full AdvantageKit and SmartDashboard telemetry including Field2d widget.

### TODO

- Add multi-camera weighting or per-camera trust thresholds if additional
  cameras are mounted.
- Consider exposing odometry-vs-fused drift as a telemetry metric for field
  calibration checks.
