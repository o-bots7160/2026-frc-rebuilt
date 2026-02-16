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
3. Stream selection uses Limelight's `stream` NetworkTables entry: `1` =
   Limelight onboard feed, `2` = external USB feed.
4. Operators can toggle streams in Elastic using
   `SmartDashboard/DriverCameraSubsystem/useUsbCameraStream`.

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

| Setting         | Units | Purpose                                                      |
| --------------- | ----- | ------------------------------------------------------------ |
| `cameraName`    | -     | NetworkTables key of the camera (for example, `"limelight"`) |
| `pipelineIndex` | index | Limelight pipeline index used while in driver mode           |
| `defaultStream` | enum  | Startup stream (`1` = onboard, `2` = USB)                    |

Runtime dashboard controls:

- `SmartDashboard/DriverCameraSubsystem/useUsbCameraStream` (`boolean`): `false`
  uses stream `1`, `true` uses stream `2`.

## Code structure

| File                                                | Purpose                                                                               |
| --------------------------------------------------- | ------------------------------------------------------------------------------------- |
| `DriverCameraSubsystem.java`                        | Subsystem that configures Limelight driver mode and manages stream selection/toggling |
| `commands/DriverCameraSubsystemCommandFactory.java` | Factory for stream-toggle and future driver camera commands                           |
| `config/DriverCameraSubsystemConfig.java`           | Configuration for camera name, startup stream, and driver pipeline                    |

## Status / TODO

### Done

- Driver-mode activation on construction.
- Dashboard toggle support for switching between Limelight onboard and USB feed.
- Disabled-subsystem guard when no camera is connected.

### TODO

<!-- TODO: fill in when hardware decisions are made -->

- Consider adding a snapshot button binding for scouting or replay review.
