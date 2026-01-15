package frc.robot.commands.factories;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drivebase.MoveFieldManualCommand;
import frc.robot.config.DriveBaseSubsystemConfig;
import frc.robot.helpers.Logger;
import frc.robot.subsystems.drivebase.DriveBaseSubsystem;
import swervelib.math.SwerveMath;

public class DriveBaseSubsystemCommandFactory extends AbstractSubsystemCommandFactory<DriveBaseSubsystem> {

    private static final double JOYSTICK_DEADBAND = 0.08;

    private final Logger         log               = Logger.getInstance(getClass());

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
     * @param driveBaseConfig drivebase configuration for scaling speeds
     * @return command that is also set as the subsystem's default
     */
    public Command setDefaultManualDriveCommand(
            Supplier<Double> forwardAxis,
            Supplier<Double> leftAxis,
            Supplier<Double> omegaAxis,
            DriveBaseSubsystemConfig driveBaseConfig) {
    Command manualDriveCommand = createMoveManualCommand(
        () -> computeScaledTranslation(forwardAxis, leftAxis, driveBaseConfig).getX(),
        () -> computeScaledTranslation(forwardAxis, leftAxis, driveBaseConfig).getY(),
        () -> scaleAngular("omega", omegaAxis.get(), driveBaseConfig));

        subsystem.setDefaultCommand(manualDriveCommand);
        return manualDriveCommand;
    }

    private Translation2d computeScaledTranslation(
            Supplier<Double> forwardAxis,
            Supplier<Double> leftAxis,
            DriveBaseSubsystemConfig config) {

        double rawForward = -forwardAxis.get();
        double rawLeft    = -leftAxis.get();

        double deadbandedForward = MathUtil.applyDeadband(rawForward, JOYSTICK_DEADBAND);
        double deadbandedLeft    = MathUtil.applyDeadband(rawLeft, JOYSTICK_DEADBAND);

        Translation2d rawVector    = new Translation2d(deadbandedForward, deadbandedLeft);
        Translation2d scaledVector = SwerveMath.scaleTranslation(rawVector, config.getTranslationScale());

        Translation2d commanded = new Translation2d(
                scaledVector.getX() * config.getMaximumLinearSpeedMetersPerSecond(),
                scaledVector.getY() * config.getMaximumLinearSpeedMetersPerSecond());

        log.recordOutput("DriverInputs/forward/raw", rawForward);
        log.recordOutput("DriverInputs/forward/deadbanded", deadbandedForward);
        log.recordOutput("DriverInputs/left/raw", rawLeft);
        log.recordOutput("DriverInputs/left/deadbanded", deadbandedLeft);
        log.recordOutput("DriverInputs/translation/scaledX", scaledVector.getX());
        log.recordOutput("DriverInputs/translation/scaledY", scaledVector.getY());
        log.recordOutput("DriverInputs/translation/commandedX", commanded.getX());
        log.recordOutput("DriverInputs/translation/commandedY", commanded.getY());

        return commanded;
    }

    private double scaleAngular(String axisName, double rawAxisValue, DriveBaseSubsystemConfig config) {
        double processed            = MathUtil.applyDeadband(-rawAxisValue, JOYSTICK_DEADBAND);
        double radiansPerSecond     = processed * config.getMaximumAngularSpeedRadiansPerSecond();

        log.recordOutput("DriverInputs/" + axisName + "/raw", rawAxisValue);
        log.recordOutput("DriverInputs/" + axisName + "/deadbanded", processed);
        log.recordOutput("DriverInputs/" + axisName + "/radiansPerSecond", radiansPerSecond);

        return radiansPerSecond;
    }
}
