package frc.robot.subsystems.drivebase.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.shared.commands.AbstractSubsystemCommandFactory;
import frc.robot.subsystems.drivebase.DriveBaseSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;

/**
 * Factory that creates drive base commands and wires default behaviors.
 */
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

    /**
     * Builds a heading-locked manual drive command that uses the heading PID controller to track a target angle while the driver retains full
     * translation control.
     *
     * @param forwardMetersPerSecondSupplier supplier of forward (field +X) velocity in meters per second
     * @param leftMetersPerSecondSupplier    supplier of leftward (field +Y) velocity in meters per second
     * @param targetHeadingRadiansSupplier   supplier of the desired field-relative heading in radians
     * @return command that continues driving with heading lock until interrupted
     */
    public MoveFieldManualWithHeadingCommand createMoveManualWithHeadingCommand(
            DoubleSupplier forwardMetersPerSecondSupplier,
            DoubleSupplier leftMetersPerSecondSupplier,
            DoubleSupplier targetHeadingRadiansSupplier) {
        return new MoveFieldManualWithHeadingCommand(
                subsystem,
                forwardMetersPerSecondSupplier,
                leftMetersPerSecondSupplier,
                targetHeadingRadiansSupplier);
    }

    /**
     * Creates a command that spins the robot 180 degrees from its heading at the moment the button is pressed. The target heading is captured once
     * via deferred proxy and held constant while the button is held. The driver retains full translation control during the spin.
     *
     * @param forwardAxis supplier providing the shaped forward stick value ([-1, 1])
     * @param leftAxis    supplier providing the shaped left stick value ([-1, 1])
     * @return deferred command that captures the current heading, adds 180 degrees, and locks onto it
     */
    public Command createSpin180Command(
            Supplier<Double> forwardAxis,
            Supplier<Double> leftAxis) {
        return Commands.deferredProxy(() -> {
            Supplier<Translation2d> translationSupplier = subsystem.mapDriverTranslationSupplier(forwardAxis, leftAxis);

            // Capture the current heading and add 180 degrees (pi radians).
            double                  currentRadians      = subsystem.getOdometryPose().getRotation().getRadians();
            double                  targetRadians       = MathUtil.angleModulus(currentRadians + Math.PI);

            return createMoveManualWithHeadingCommand(
                    () -> translationSupplier.get().getX(),
                    () -> translationSupplier.get().getY(),
                    () -> targetRadians);
        });
    }

    /**
     * Creates a command that snaps the robot to the nearest field-facing orientation (0 degrees or 180 degrees field-relative). If the robot is
     * already within the configured rotation tolerance of the nearest orientation, the command picks the opposite one instead. The target is captured
     * once when the button is pressed and held constant while held.
     * <p>
     * Field-facing means aligned with the field's X axis: 0 degrees faces the red alliance wall, 180 degrees faces the blue alliance wall.
     * </p>
     *
     * @param forwardAxis supplier providing the shaped forward stick value ([-1, 1])
     * @param leftAxis    supplier providing the shaped left stick value ([-1, 1])
     * @return deferred command that snaps to the nearest (or opposite) field-facing heading
     */
    public Command createSnapToFieldFacingCommand(
            Supplier<Double> forwardAxis,
            Supplier<Double> leftAxis) {
        return Commands.deferredProxy(() -> {
            Supplier<Translation2d> translationSupplier = subsystem.mapDriverTranslationSupplier(forwardAxis, leftAxis);

            double                  currentRadians      = subsystem.getOdometryPose().getRotation().getRadians();
            double                  facingMarginRadians = subsystem.getFieldFacingMarginRadians();

            // Determine which field-facing orientation is closest.
            double                  forwardRadians      = 0.0;
            double                  backwardRadians     = Math.PI;

            // Compute the shortest angular distance to each candidate.
            double                  distanceToForward   = Math.abs(MathUtil.angleModulus(currentRadians - forwardRadians));
            double                  distanceToBackward  = Math.abs(MathUtil.angleModulus(currentRadians - backwardRadians));

            double                  targetRadians;
            if (distanceToForward <= facingMarginRadians) {
                // Already facing forward; flip to backward.
                targetRadians = backwardRadians;
            } else if (distanceToBackward <= facingMarginRadians) {
                // Already facing backward; flip to forward.
                targetRadians = forwardRadians;
            } else {
                // Not facing either direction; snap to whichever is closer.
                targetRadians = distanceToForward <= distanceToBackward ? forwardRadians : backwardRadians;
            }

            // Normalize the target so the PID wraps correctly.
            double normalizedTarget = MathUtil.angleModulus(targetRadians);

            return createMoveManualWithHeadingCommand(
                    () -> translationSupplier.get().getX(),
                    () -> translationSupplier.get().getY(),
                    () -> normalizedTarget);
        });
    }
}
