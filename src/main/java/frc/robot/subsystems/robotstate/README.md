# Robot State subsystem

## Overview

The Robot State subsystem is the **single authoritative source of robot
[pose](../../GLOSSARY.md#pose)** for the entire codebase. Every command and
subsystem that needs to know where the robot is on the field reads from this
subsystem rather than querying sensors directly. It does not run its own filter
— it coordinates data flow between pose sources (cameras, future sensors) and
the drivebase's internal pose estimator, then serves the fused result to
consumers.

## How it works

### Data flow

```
Camera A ──► RobotStateSubsystem ──(forward)──► DriveBase (YAGSL estimator)
Camera B ──►        │                                  │
Future   ──►        │                        getPose() (fused)
                    │◄─────────────────────────────────┘
                    │
                    ▼
              Turret, PathPlanner, etc.
```

1. **Pose sources** (cameras, LIDAR, future sensors) push measurements into
   `addVisionMeasurement(pose, timestamp, stdDevs)`.
2. This subsystem forwards those measurements to the drivebase's
   [YAGSL](../../GLOSSARY.md#yagsl) estimator through a
   `VisionMeasurementConsumer` wired in `RobotContainer`.
3. Each cycle, the subsystem **pulls** the latest fused
   [pose](../../GLOSSARY.md#pose) from the drivebase via a `Supplier<Pose2d>`
   and publishes it as `getEstimatedPose()`.

### Pose fusion explained

The drivebase's [YAGSL](../../GLOSSARY.md#yagsl) `SwerveDrivePoseEstimator` runs
a [Kalman filter](../../GLOSSARY.md#kalman-filter) — a math technique that
blends two imperfect data sources into a single, more accurate estimate.

- **[Odometry](../../GLOSSARY.md#odometry)** updates every code cycle (50 Hz).
  It is smooth and responsive but slowly drifts because wheels slip and
  [encoders](../../GLOSSARY.md#encoder) accumulate small errors.
- **Vision** updates less frequently and has higher
  [latency](../../GLOSSARY.md#latency), but each fix is an absolute field
  position derived from [AprilTags](../../GLOSSARY.md#apriltag).

The Kalman filter trusts odometry for short-term motion and uses vision to nudge
the estimate back toward the true position. The
[confidence](../../GLOSSARY.md#confidence) values (standard deviations) attached
to each vision measurement tell the filter how much to trust that particular
fix.

### Adding a new pose source

Wire the new subsystem's output to `robotStateSubsystem::addVisionMeasurement`
in `RobotContainer`. No other code changes are needed:

```java
newSensorSubsystem = new SomeSubsystem(config, robotStateSubsystem::addVisionMeasurement);
```

## Configuration

Settings live in `subsystems.json` under `robotStateSubsystem`.

| Setting              | Purpose                                                          |
| -------------------- | ---------------------------------------------------------------- |
| `enabled`            | Toggles the subsystem on or off                                  |
| `enableVisionFusion` | Gates whether vision measurements are forwarded to the estimator |

## Code structure

| File                                    | Purpose                                                                                                         |
| --------------------------------------- | --------------------------------------------------------------------------------------------------------------- |
| `RobotStateSubsystem.java`              | Coordination layer — accepts vision measurements, forwards to drivebase, pulls fused pose, owns Field2d display |
| `config/RobotStateSubsystemConfig.java` | Subsystem config with enable and vision-fusion toggles                                                          |
| `io/RobotStateIO.java`                  | AdvantageKit IO interface for logging robot state inputs                                                        |

## Status / TODO

### Done

- Single authoritative pose source for all consumers.
- Vision measurement forwarding with configurable fusion toggle.
- Odometry reset coordination so drivebase and estimator stay aligned.
- Field2d display for dashboard visualization.
- AdvantageKit logging of estimated and vision poses.

### TODO

- Add more pose sources as new sensors come online.
- Expose alliance-relative pose helpers for autonomous planning.
