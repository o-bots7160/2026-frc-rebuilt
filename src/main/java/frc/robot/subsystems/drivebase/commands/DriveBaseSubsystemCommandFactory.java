package frc.robot.subsystems.drivebase.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.shared.commands.AbstractSubsystemCommandFactory;
import frc.robot.subsystems.drivebase.DriveBaseSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;

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
     * @param forwardAxis supplier providing forward stick value
     * @param leftAxis    supplier providing left stick value
     * @param omegaAxis   supplier providing rotation stick value
     * @return command that is also set as the subsystem's default
     */
    public Command setDefaultManualDriveCommand(
            Supplier<Double> forwardAxis,
            Supplier<Double> leftAxis,
            Supplier<Double> omegaAxis) {
        Supplier<Translation2d> translationSupplier = subsystem.mapDriverTranslationSupplier(forwardAxis, leftAxis);
        Supplier<Double>        omegaSupplier       = subsystem.mapDriverOmegaSupplier(omegaAxis);

        Command                 manualDriveCommand  = createMoveManualCommand(
                () -> translationSupplier.get().getX(),
                () -> translationSupplier.get().getY(),
                () -> omegaSupplier.get());

        subsystem.setDefaultCommand(manualDriveCommand);
        return manualDriveCommand;
    }

    /**
     * Creates a SysId command that exercises the drive motors using the configured YAGSL characterization routine.
     *
     * @return command suitable for binding to a dashboard/button for on-robot testing
     */
    public Command createDriveSysIdCommand() {
        if (subsystem.isSubsystemDisabled()) {
            return Commands.print("Drive SysId skipped: drive base disabled.");
        }

        SwerveDrive drive = subsystem.getSwerveDrive();
        if (drive == null) {
            return Commands.print("Drive SysId skipped: swerve drive not configured.");
        }

        SysIdRoutine routine = SwerveDriveTest.setDriveSysIdRoutine(new SysIdRoutine.Config(), subsystem, drive, 6.0, false);
        return SwerveDriveTest.generateSysIdCommand(routine, 3.0, 3.0, 3.0);
    }

    /**
     * Creates a SysId command that exercises the steer (angle) motors using the configured YAGSL characterization routine.
     *
     * @return command suitable for binding to a dashboard/button for on-robot testing
     */
    public Command createAngleSysIdCommand() {
        if (subsystem.isSubsystemDisabled()) {
            return Commands.print("Angle SysId skipped: drive base disabled.");
        }

        SwerveDrive drive = subsystem.getSwerveDrive();
        if (drive == null) {
            return Commands.print("Angle SysId skipped: swerve drive not configured.");
        }

        SysIdRoutine routine = SwerveDriveTest.setAngleSysIdRoutine(new SysIdRoutine.Config(), subsystem, drive);
        return SwerveDriveTest.generateSysIdCommand(routine, 3.0, 4.0, 4.0);
    }
}
