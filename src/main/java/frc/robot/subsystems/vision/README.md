# Vision subsystem

## Role (game/mechanical)

Vision is used in a few different ways here with AprilTag processing being
the first priority.  AprilTagVisionSubsystem gives the robot field awareness.
Uses AprilTags to estimate our pose on the field so we can accurately control
the turret for aiming and shooting.

The DriverCameraSubsystem provides a field view from the robot perspective.
Having the robot perspective may be necessary from certain driver stations
for navigating out of jams or to find FUEL.

Some vision stretch goals would be a birds-eye view of the robot and object
detection to locate FUEL.  Switching between the driver camera and birds-eye
view would be ideal for navigating and FUEL collection when the driver can't
see the robot.  The birds-eye view would likely be better to help the driver
determine which way to turn to collect cells.  Similarly, object detection
can help guide the driver to the largest deposit of FUEL.

## Control and configuration (programming)

Done
- Estimates robot pose relative to the field using tag observations.
- Tracks goals or pickup spots and shares angles to the turret and drivebase.
- Flags target confidence so commands can decide when to trust vision over
  odometry.
- Publishes pose and target info to AdvantageKit; drivebase can fuse it with
  odometry.
- Document camera mounting offsets in config files to keep transforms correct.

TODO
- Run latency compensation so turret and shooter aim use current data.
- Provide a toggle for driver-assist aiming vs. pure manual control.
- In sim, feed synthetic tag data so autonomous paths can be tested without
  cameras.
- Expose ready/valid signals so commands only act on good frames.

## Code structure and maintenance

- Classes (planned): vision subsystem, pipeline/camera IO, and adapters for
  turret/drivebase consumers.
- Reviewer notes: keep transforms in config, not code; gate outputs with
  confidence to avoid bad frames driving commands.

## TODO

- Select camera hardware and finalize AprilTag pipeline; add calibration steps
  and config fields once hardware is mounted.
- Add target config so turret can be aimed at hub versus targets.
