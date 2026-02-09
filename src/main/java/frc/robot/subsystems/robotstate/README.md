# Robot State Subsystem

The Robot State subsystem is the **single authoritative source of robot pose**
for the entire codebase. Every command and subsystem reads pose data through
this subsystem rather than querying sensors directly.

## Architecture

Pose fusion is **delegated** to the drivebase's internal YAGSL
`SwerveDrivePoseEstimator`, which provides Kalman-filter-based fusion with
latency compensation and uncertainty weighting. This subsystem acts as the
coordination layer:

1. **Pose sources** (cameras, LIDAR, future sensors) push measurements into
   `addVisionMeasurement(pose, timestamp, stdDevs)`.
2. This subsystem forwards those measurements to the drivebase's YAGSL estimator
   through a `VisionMeasurementConsumer` wired in `RobotContainer`.
3. Each cycle, the subsystem **pulls** the latest fused pose from the drivebase
   via a `Supplier<Pose2d>` and publishes it as `getEstimatedPose()`.

```
Camera A ──► RobotStateSubsystem ──(forward)──► DriveBase (YAGSL estimator)
Camera B ──►        │                                  │
Future   ──►        │                        getPose() (fused)
                    │◄─────────────────────────────────┘
                    │
                    ▼
              Turret, PathPlanner, etc.
```

## Responsibilities

- Accept vision measurements from any number of pose sources.
- Forward measurements to the drivebase's YAGSL pose estimator for fusion.
- Pull the fused pose each cycle and serve it to consumers.
- Provide read-only pose access to other subsystems (turret, shooter, autonomous
  planners).
- Log estimated and vision poses to AdvantageKit for debugging.
- Own the Field2d display so dashboards see the fused pose.
- Coordinate odometry resets so the drivebase and robot state stay aligned.
- Gate vision fusion via the `enableVisionFusion` config toggle.

## Adding a new pose source

Wire the new subsystem's output to `robotStateSubsystem::addVisionMeasurement`
in `RobotContainer`. No other code changes are needed:

```java
newSensorSubsystem = new SomeSubsystem(config, robotStateSubsystem::addVisionMeasurement);
```

## Usage notes

- Commands should depend on this subsystem for field pose data rather than
  talking directly to multiple sensors.
- Vision subsystems should inject measurements into this subsystem using the
  `addVisionMeasurement` API.
- The drivebase is the fusion engine; this subsystem is the coordination and
  read-only authority layer.
- Use `setOdometryResetConsumer` to keep odometry reset behavior in one place.
