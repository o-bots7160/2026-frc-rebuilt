package frc.robot.commands.factories;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drivebase.MoveFieldManualCommand;
import frc.robot.subsystems.drivebase.DriveBaseSubsystem;

public class DriveBaseSubsystemCommandFactory extends AbstractSubsystemCommandFactory<DriveBaseSubsystem> {

    /**
     * Creates a factory that produces commands operating on the provided drive base subsystem.
     *
     * @param subsystem drive base subsystem instance to be shared by generated commands
     */
    public DriveBaseSubsystemCommandFactory(DriveBaseSubsystem subsystem) {
        super(subsystem);
    }

    /**
     * Builds a field-relative manual driving command driven by the supplied control inputs.
     *
     * @param forwardMetersPerSecondSupplier supplier of forward (field +X) velocity in meters per second
     * @param leftMetersPerSecondSupplier    supplier of leftward (field +Y) velocity in meters per second
     * @param ccwRadiansPerSecondSupplier    supplier of counter-clockwise angular velocity in radians per second
     * @return command that continues driving until interrupted
     */
    public MoveFieldManualCommand createMoveManualCommand(
            DoubleSupplier forwardMetersPerSecondSupplier,
            DoubleSupplier leftMetersPerSecondSupplier,
            DoubleSupplier ccwRadiansPerSecondSupplier) {
        return new MoveFieldManualCommand(subsystem, forwardMetersPerSecondSupplier, leftMetersPerSecondSupplier, ccwRadiansPerSecondSupplier);
    }

    /**
     * Builds and sets the default manual drive command using a driver controller.
     *
     * @param forwardAxis     supplier providing forward stick value
     * @param leftAxis        supplier providing left stick value
     * @param omegaAxis       supplier providing rotation stick value
     * @return command that is also set as the subsystem's default
     */
    public Command setDefaultManualDriveCommand(
            Supplier<Double> forwardAxis,
            Supplier<Double> leftAxis,
            Supplier<Double> omegaAxis) {
    Supplier<Translation2d> translationSupplier = subsystem.mapDriverTranslationSupplier(forwardAxis, leftAxis);
    Supplier<Double>        omegaSupplier       = subsystem.mapDriverOmegaSupplier(omegaAxis);

    Command manualDriveCommand = createMoveManualCommand(
        () -> translationSupplier.get().getX(),
        () -> translationSupplier.get().getY(),
        () -> omegaSupplier.get());

        subsystem.setDefaultCommand(manualDriveCommand);
        return manualDriveCommand;
    }
}
