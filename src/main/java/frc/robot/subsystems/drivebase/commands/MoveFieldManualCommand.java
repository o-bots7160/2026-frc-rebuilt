package frc.robot.subsystems.drivebase.commands;

import java.util.function.DoubleSupplier;

import frc.robot.shared.commands.AbstractSubsystemCommand;
import frc.robot.subsystems.drivebase.DriveBaseSubsystem;

/**
 * Drives the robot field-relative using continuously supplied chassis speeds. Forward/left translation and counter-clockwise rotation requests
 * typically come from driver controls.
 */
public class MoveFieldManualCommand extends AbstractSubsystemCommand<DriveBaseSubsystem> {

    private final DoubleSupplier forwardMetersPerSecondSupplier;

    private final DoubleSupplier leftMetersPerSecondSupplier;

    private final DoubleSupplier ccwRadiansPerSecondSupplier;

    /**
     * Creates a manual drive command that issues field-relative chassis speeds.
     *
     * @param driveBaseSubsystem             drive base subsystem to command
     * @param forwardMetersPerSecondSupplier supplier providing forward (positive X) velocity in meters per second
     * @param leftMetersPerSecondSupplier    supplier providing leftward (positive Y) velocity in meters per second
     * @param ccwRadiansPerSecondSupplier    supplier providing counter-clockwise angular velocity in radians per second
     */
    public MoveFieldManualCommand(
            DriveBaseSubsystem driveBaseSubsystem,
            DoubleSupplier forwardMetersPerSecondSupplier,
            DoubleSupplier leftMetersPerSecondSupplier,
            DoubleSupplier ccwRadiansPerSecondSupplier) {
        super(driveBaseSubsystem);
        this.forwardMetersPerSecondSupplier = forwardMetersPerSecondSupplier;
        this.leftMetersPerSecondSupplier    = leftMetersPerSecondSupplier;
        this.ccwRadiansPerSecondSupplier    = ccwRadiansPerSecondSupplier;
    }

    /**
     * Continuously drives the robot using the supplied field-relative speed requests.
     */
    @Override
    public void execute() {
        subsystem.driveFieldRelative(
                forwardMetersPerSecondSupplier.getAsDouble(),
                leftMetersPerSecondSupplier.getAsDouble(),
                -ccwRadiansPerSecondSupplier.getAsDouble());
    }

    /**
     * Stops and locks the drivebase once manual control ends or is interrupted.
     *
     * @param interrupted true when the command was cancelled before normal completion
     */
    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }

    /**
     * Clears any pose targets so manual control can fully own the drivebase.
     */
    @Override
    protected void onInitialize() {
        // no op
    }
}
