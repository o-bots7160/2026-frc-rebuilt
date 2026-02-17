# Climber subsystem

## Overview

The Climber subsystem raises the robot onto tower rungs for endgame points in
REBUILT. It extends and retracts arms or hooks for staged climbs: latch a lower
rung, transfer weight, then reach higher. Once the robot is hanging, the climber
holds position under load to prevent backdrive while waiting for the buzzer.

Two front-facing PlayingWithFusion Time of Flight sensors detect and align the
robot with the tower before climbing begins. The subsystem extends the
set-and-seek pattern so motor-driven stage positions can be added once hardware
is selected.

## How it works

The climber uses two front-facing Time of Flight (ToF) sensors mounted
side-by-side to measure the distance to the tower. When both sensors read below
the configured maximum detection distance, the tower is considered detected.
When the absolute difference between left and right readings falls within the
alignment tolerance, the robot is centered and square to the tower.

Motor functionality is not yet implemented. The subsystem currently runs with a
`DisabledMotor` and focuses on sensor telemetry. Once motor hardware is
selected, the `buildMotor()` method in `ClimberSubsystem` should be updated to
construct the real motor, following the turret pattern.

Planned behaviors (pending motor implementation):

- **Stage detection** — encoder positions at each rung so the driver sees live
  progress on the dashboard.
- **Scripted sequence** — an autonomous climb command runs stages in order with
  clear abort paths if something goes wrong.
- **Manual override** — a backup mode where the operator directly controls
  extension and retraction.
- **Hold under load** — once latched, the motors brake and current-limit to hold
  the robot's weight without overheating.

## Configuration

Settings are stored in `subsystems.json` under `"climberSubsystem"`:

| Setting                                                   | Units        | Purpose                                       |
| --------------------------------------------------------- | ------------ | --------------------------------------------- |
| `enabled`                                                 | —            | Master enable flag                            |
| `climberMotorConfig.motorCanId`                           | —            | CAN ID for the climber motor (placeholder)    |
| `climberSensorConfig.leftSensorCanId`                     | —            | CAN ID for the left ToF sensor                |
| `climberSensorConfig.rightSensorCanId`                    | —            | CAN ID for the right ToF sensor               |
| `climberSensorConfig.alignmentToleranceMillimeters`       | millimeters  | Max left/right difference for alignment       |
| `climberSensorConfig.maximumDetectionDistanceMillimeters` | millimeters  | Readings beyond this mean no tower            |
| `climberSensorConfig.sampleTimeMilliseconds`              | milliseconds | ToF ranging sample period                     |
| `minimumSetpointDegrees` / `maximumSetpointDegrees`       | degrees      | Motor travel limits (placeholder)             |
| `kP`, `kI`, `kD`, `kS`, `kV`, `kA`                        | —            | PID and feedforward gains (placeholder zeros) |

## Code structure

| File                                           | Purpose                                                             |
| ---------------------------------------------- | ------------------------------------------------------------------- |
| `ClimberSubsystem.java`                        | Subsystem with ToF sensor logic and disabled motor placeholder      |
| `commands/MoveClimberToPositionCommand.java`   | Set-and-seek command for stage positions (placeholder)              |
| `commands/ClimberSubsystemCommandFactory.java` | Factory for climber commands                                        |
| `config/ClimberSubsystemConfig.java`           | Root config extending set-and-seek config                           |
| `config/ClimberMotorConfig.java`               | Motor config placeholder                                            |
| `config/ClimberSensorConfig.java`              | ToF sensor CAN IDs, thresholds, and sample timing                   |
| `io/ClimberSensorIO.java`                      | IO interface for ToF sensors with `@AutoLog` inputs                 |
| `io/ClimberSensorIOReal.java`                  | Real hardware implementation using PlayingWithFusion `TimeOfFlight` |
| `io/ClimberSensorIOSim.java`                   | Simulation implementation with configurable default values          |

## Dashboard and telemetry

- **Elastic** — "Climber" tab shows sensor CAN IDs, live distance readings,
  tower detection / alignment status, sensor connection indicators, and PID/FF
  gain fields.
- **AdvantageScope** — "Climber ToF Sensors" tab graphs left/right distances and
  difference with boolean stripes for tower detection, alignment, and sensor
  connection. "Climber Motor Tuning" tab is stubbed with the same layout as the
  turret tuning tab.

## Status / TODO

### Done

- README and folder structure created.
- PlayingWithFusion vendor dependency added.
- ToF sensor IO interface with real and sim implementations.
- Sensor config with tunable CAN IDs, alignment tolerance, and detection
  distance.
- Climber subsystem exposing `getLeftDistanceMillimeters()`,
  `getRightDistanceMillimeters()`, `isAlignedToTower()`, and
  `isTowerDetected()`.
- Wired into `SubsystemsConfig`, all three JSON config files, `RobotContainer`,
  and `TriggerBindings`.
- Elastic dashboard tab and AdvantageScope tabs for sensor and motor telemetry.

### TODO

- Select motor hardware and update `buildMotor()`.
- Implement staged-climb command sequence with abort logic.
- Add manual override command for operator recovery.
- Wire operator controller buttons to climber commands in `TriggerBindings`.
