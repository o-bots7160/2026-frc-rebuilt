package frc.robot.subsystems.drivercameravision.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.commands.AbstractSubsystemCommandFactory;
import frc.robot.subsystems.drivercameravision.DriverCameraSubsystem;

/**
 * Factory that creates commands for the driver camera subsystem.
 */
public class DriverCameraSubsystemCommandFactory extends AbstractSubsystemCommandFactory<DriverCameraSubsystem> {

    /**
     * Creates a factory that produces commands operating on the provided driver camera subsystem.
     *
     * @param subsystem driver camera subsystem instance that commands created by this factory will control
     */
    public DriverCameraSubsystemCommandFactory(DriverCameraSubsystem subsystem) {
        super(subsystem);
    }

    /**
     * Builds a one-shot command that toggles the active driver stream between the Limelight onboard camera and the
     * external USB camera.
     *
     * @return command for dashboard or button bindings
     */
    public Command createToggleStreamCommand() {
        return Commands.runOnce(subsystem::toggleStreamMode, subsystem);
    }
}
