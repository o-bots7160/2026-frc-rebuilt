# Robot State Subsystem

The Robot State subsystem centralizes pose data so every command and subsystem
reads the same fused view of the robot on the field. It is the place to combine
drivebase odometry, vision measurements, and field layout transforms into one
authoritative pose estimate.

## Responsibilities

- Track the latest odometry pose supplied by the drivebase.
- Accept vision measurements and fuse them into the pose estimate.
- Provide read-only pose access to other subsystems (turret, shooter, autonomous
  planners).
- Log estimated, odometry, and vision poses to AdvantageKit for debugging.
- Own the Field2d display so dashboards see the fused pose.
- Coordinate odometry resets so the drivebase and robot state stay aligned.

## Usage notes

- Commands should depend on this subsystem for field pose data rather than
  talking directly to multiple sensors.
- Vision subsystems should inject measurements into this subsystem using the
  `addVisionMeasurement` API.
- The drivebase should be the single source of odometry pose updates.
- Use `setOdometryResetConsumer` to keep odometry reset behavior in one place.
