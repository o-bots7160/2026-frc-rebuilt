# Harvester Subsystem

The harvester arm swings the intake rollers between two named positions using a
profiled motor. It is completely independent of the intake rollers themselves,
which live in the `intake/` subsystem.

## Positions

| Name     | Description                                            |
| -------- | ------------------------------------------------------ |
| Stowed   | Arm upright inside the robot perimeter (match-start)   |
| Deployed | Arm lowered outside the frame to collect Fuel from the |
|          | floor                                                  |

## Key classes

| File                                             | Purpose                                      |
| ------------------------------------------------ | -------------------------------------------- |
| `HarvesterSubsystem.java`                        | Set-and-seek subsystem with deploy/stow API  |
| `config/HarvesterSubsystemConfig.java`           | Positions, PID, profile, and tolerance       |
| `config/HarvesterMotorConfig.java`               | CAN ID, gear ratio, soft limits              |
| `devices/HarvesterMotor.java`                    | SparkMax wrapper with brake mode and gearing |
| `devices/HarvesterSimMotor.java`                 | Simulation motor for profile testing         |
| `commands/MoveHarvesterToPositionCommand.java`   | Profiled move command (supplier or fixed)    |
| `commands/HarvesterSubsystemCommandFactory.java` | Factory for deploy, stow, and move commands  |

## Configuration

All angles are stored in degrees in `subsystems.json` (and sim/test variants).
The subsystem config provides tunable getters so positions and profile limits
can be adjusted on the fly from Elastic without redeploying.

## Relationship to the intake

The harvester and intake are separate subsystems sharing no direct references.
In `RobotContainer`, trigger bindings compose them: pressing a button deploys
the harvester arm **and** spins the intake rollers simultaneously through
parallel command groups.
