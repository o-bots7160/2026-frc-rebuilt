package frc.robot.subsystems.drivebase.commands;

import java.util.function.DoubleSupplier;

import frc.robot.shared.commands.AbstractSubsystemCommand;
import frc.robot.subsystems.drivebase.DriveBaseSubsystem;

/**
 * Drives the robot field-relative using driver-supplied translation while locking the heading to a PID-controlled target angle. The driver retains
 * full left-stick control over forward and strafe, but the rotation axis is overridden by the heading PID controller to drive toward the target.
 * <p>
 * This command runs until interrupted (designed for {@code whileTrue} bindings). When the button is released, the default manual drive command
 * resumes and the driver regains manual omega control.
 * </p>
 */
public class MoveFieldManualWithHeadingCommand extends AbstractSubsystemCommand<DriveBaseSubsystem> {

    /**
     * Supplier providing field-forward velocity in meters per second.
     */
    private final DoubleSupplier forwardMetersPerSecondSupplier;

    /**
     * Supplier providing field-left velocity in meters per second.
     */
    private final DoubleSupplier leftMetersPerSecondSupplier;

    /**
     * Supplier providing the desired field-relative heading in radians.
     */
    private final DoubleSupplier targetHeadingRadiansSupplier;

    /**
     * Creates a heading-locked manual drive command.
     *
     * @param driveBaseSubsystem             drive base subsystem to command
     * @param forwardMetersPerSecondSupplier supplier providing forward (positive X) velocity in meters per second
     * @param leftMetersPerSecondSupplier    supplier providing leftward (positive Y) velocity in meters per second
     * @param targetHeadingRadiansSupplier   supplier providing the desired heading in radians (counter-clockwise positive)
     */
    public MoveFieldManualWithHeadingCommand(
            DriveBaseSubsystem driveBaseSubsystem,
            DoubleSupplier forwardMetersPerSecondSupplier,
            DoubleSupplier leftMetersPerSecondSupplier,
            DoubleSupplier targetHeadingRadiansSupplier) {
        super(driveBaseSubsystem);
        this.forwardMetersPerSecondSupplier = forwardMetersPerSecondSupplier;
        this.leftMetersPerSecondSupplier    = leftMetersPerSecondSupplier;
        this.targetHeadingRadiansSupplier   = targetHeadingRadiansSupplier;
    }

    /**
     * Drives field-relative with PID-controlled heading each cycle.
     */
    @Override
    public void execute() {
        subsystem.driveFieldRelativeWithHeading(
                forwardMetersPerSecondSupplier.getAsDouble(),
                leftMetersPerSecondSupplier.getAsDouble(),
                targetHeadingRadiansSupplier.getAsDouble());
    }

    /**
     * Stops and locks the drivebase once the heading lock ends or is interrupted.
     *
     * @param interrupted true when the command was cancelled before normal completion
     */
    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }

    /**
     * Resets the heading PID controller so stale integral error does not produce a large initial output.
     */
    @Override
    protected void onInitialize() {
        subsystem.resetHeadingController();
    }
}
