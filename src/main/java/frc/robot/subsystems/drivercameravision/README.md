# Driver Camera Vision subsystem

## Overview

The Driver Camera subsystem manages a forward-facing camera that gives the
driver and operator a live video feed on the Driver Station dashboard. Unlike
the [AprilTag Vision subsystem](../apriltagvision/README.md), this camera does
not calculate any [pose](../../GLOSSARY.md#pose) data — it is purely for
visibility so drivers can align the robot during teleop.

## How it works

1. A camera (typically a Limelight or USB camera) streams video to the Driver
   Station through NetworkTables.
2. `DriverCameraSubsystem` puts the camera into "driver mode" on startup. In
   driver mode the camera acts as a plain video pass-through — no vision
   processing, no overlays — which keeps [latency](../../GLOSSARY.md#latency)
   low and the video feed clear.
3. The subsystem does not publish or consume any position data. It is a
   fire-and-forget setup: initialize once, and the video stream runs for the
   rest of the match.

### Why a separate subsystem?

Even though it has no complex logic, wrapping the camera in a subsystem gives us
the standard disabled-subsystem lifecycle:

- If the camera hardware is missing, the subsystem disables itself and the rest
  of the robot keeps running.
- Logging and telemetry come for free through `AbstractSubsystem`.
- `RobotContainer` treats it the same as every other subsystem, keeping wiring
  predictable.

## Configuration

Settings live in `subsystems.json` under `driverCameraSubsystem`.

| Setting      | Units | Purpose                                                      |
| ------------ | ----- | ------------------------------------------------------------ |
| `cameraName` | —     | NetworkTables key of the camera (e.g., `"limelight-driver"`) |

No tunable numbers are exposed because the camera is configured entirely on the
Limelight web interface (resolution, exposure, LED mode). The subsystem only
needs the name to connect.

## Code structure

| File                                      | Purpose                                                                          |
| ----------------------------------------- | -------------------------------------------------------------------------------- |
| `DriverCameraSubsystem.java`              | Subsystem that puts the camera into driver mode and holds the network connection |
| `config/DriverCameraSubsystemConfig.java` | Configuration for camera name and enabled flag                                   |

## Status / TODO

### Done

- Driver-mode activation on construction.
- Disabled-subsystem guard when no camera is connected.

### TODO

<!-- TODO: fill in when hardware decisions are made -->

- Add a NetworkTables boolean that lets operators toggle crosshairs on/off
  mid-match.
- Consider adding a snapshot button binding for scouting or replay review.
