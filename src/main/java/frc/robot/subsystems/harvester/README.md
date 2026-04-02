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
| Sweep    | Arm raised (~100°) to push Fuel from the back of the   |
|          | hopper toward the shooting array                       |

## Key classes

| File                                                 | Purpose                                       |
| ---------------------------------------------------- | --------------------------------------------- |
| `HarvesterSubsystem.java`                            | Set-and-seek subsystem with deploy/stow API   |
| `config/HarvesterSubsystemConfig.java`               | Positions, PID, profile, and tolerance        |
| `shared/config/MotorConfig.java`                     | CAN ID, gear ratio, soft limits (inherited)   |
| `devices/HarvesterMotor.java`                        | SparkMax wrapper with brake mode and gearing  |
| `devices/HarvesterSimMotor.java`                     | Simulation motor for profile testing          |
| `commands/MoveHarvesterToPositionCommand.java`       | Profiled move command (supplier or fixed)     |
| `commands/HoldHarvesterDeployedPositionCommand.java` | Default command that holds deployed position  |
| `commands/HarvesterSubsystemCommandFactory.java`     | Factory for deploy, stow, hold, and move cmds |

## Configuration

All angles are stored in degrees in `subsystems.json` (and sim/test variants).
The subsystem config provides tunable getters so positions and profile limits
can be adjusted on the fly from Elastic without redeploying.

## Relationship to the intake

The harvester and intake are separate subsystems sharing no direct references.
In `RobotContainer`, trigger bindings compose them: pressing a button deploys
the harvester arm **and** spins the intake rollers simultaneously through
parallel command groups.

## Deployed position hold

When the arm is deployed for intake, incoming Fuel can push it away from the
target position. The `HoldHarvesterDeployedPositionCommand` (set as the default
command) detects this drift and re-engages the motor to drive the arm back to
the deployed angle using a smooth profiled motion. Once the arm settles, the
motor stops to avoid burning the motor with continuous current. When the arm is
not in deployed mode (stowed or transitioning), the command idles passively.

The drift threshold is configured by `deployedHoldToleranceDegrees` in
`subsystems.json` and is tunable via SmartDashboard at runtime.
