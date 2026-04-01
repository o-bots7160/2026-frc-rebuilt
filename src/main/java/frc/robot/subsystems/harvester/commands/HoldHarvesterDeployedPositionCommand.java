package frc.robot.subsystems.harvester.commands;

import frc.robot.shared.commands.AbstractSubsystemCommand;
import frc.robot.subsystems.harvester.HarvesterSubsystem;
import frc.robot.subsystems.harvester.config.HarvesterSubsystemConfig;

/**
 * Default command that monitors the harvester arm while deployed and re-engages the motor if something pushes it away from the deployed position.
 * <p>
 * During intake, a ball entering the mechanism can push the arm upward. This command detects the drift and drives the arm back to the configured
 * deployed angle using a smooth profiled motion. Once the arm settles, the motor stops to prevent unnecessary current draw. When the arm is not in
 * deployed mode (e.g., stowed or mid-transition), the command idles passively with no motor output.
 * </p>
 */
public class HoldHarvesterDeployedPositionCommand extends AbstractSubsystemCommand<HarvesterSubsystem> {

    /** Subsystem configuration for reading deployed position and hold tolerance. */
    private final HarvesterSubsystemConfig config;

    /** True while the motor is actively driving back to the deployed position. */
    private boolean seeking;

    /**
     * Creates the hold command for the given harvester subsystem.
     *
     * @param subsystem harvester subsystem to monitor and correct
     */
    public HoldHarvesterDeployedPositionCommand(HarvesterSubsystem subsystem) {
        super(subsystem);
        this.config = subsystem.getConfig();
    }

    @Override
    public void execute() {
        if (!subsystem.isInDeployedMode()) {
            // Not deployed — stop motor if we were seeking and idle.
            if (seeking) {
                subsystem.handleSeekInterrupted();
                seeking = false;
            }
            return;
        }

        if (seeking) {
            // Actively driving back to the deployed position.
            subsystem.seekTarget();
            if (subsystem.isProfileSettled()) {
                subsystem.handleSeekInterrupted();
                seeking = false;
            }
        } else {
            // Monitoring — check whether the arm has drifted beyond tolerance.
            double measuredDegrees  = subsystem.getMeasuredPositionDegrees();
            double deployedDegrees  = config.getDeployedPositionDegrees();
            double toleranceDegrees = config.getDeployedHoldToleranceDegrees();

            if (Math.abs(measuredDegrees - deployedDegrees) > toleranceDegrees) {
                log.info("Arm drifted to " + String.format("%.1f", measuredDegrees)
                        + "° — re-engaging to deployed position");
                subsystem.settleAtCurrentPosition();
                subsystem.deployArm();
                seeking = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (seeking) {
            subsystem.handleSeekInterrupted();
        }
    }

    @Override
    public boolean isFinished() {
        // Default command — runs until interrupted.
        return false;
    }

    @Override
    protected void onInitialize() {
        seeking = false;
    }
}
