package frc.robot.subsystems.drivebase.commands;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.shared.commands.AbstractSubsystemCommandFactory;
import frc.robot.shared.config.SysIdRoutineConfig;
import frc.robot.subsystems.drivebase.DriveBaseSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * Factory that creates drive base commands and wires default behaviors.
 */
public class DriveBaseSubsystemCommandFactory extends AbstractSubsystemCommandFactory<DriveBaseSubsystem> {

    /** Module index for the front-left swerve module in the YAGSL modules array. */
    public static final int MODULE_FRONT_LEFT  = 0;

    /** Module index for the front-right swerve module in the YAGSL modules array. */
    public static final int MODULE_FRONT_RIGHT = 1;

    /** Module index for the back-left swerve module in the YAGSL modules array. */
    public static final int MODULE_BACK_LEFT   = 2;

    /** Module index for the back-right swerve module in the YAGSL modules array. */
    public static final int MODULE_BACK_RIGHT  = 3;

    /** Human-readable names for each module index, matching the YAGSL swervedrive.json ordering. */
    private static final String[] MODULE_NAMES = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};

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
     * Creates a SysId command that exercises all four drive motors simultaneously using the configured YAGSL
     * characterization routine. Timing is read from the drivebase SysId config.
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

        SysIdRoutineConfig sysIdConfig = subsystem.getConfig().sysId;
        SysIdRoutine.Config routineConfig = new SysIdRoutine.Config(
                Volts.per(edu.wpi.first.units.Units.Second).of(sysIdConfig.getRampRateVoltsPerSecond()),
                Volts.of(sysIdConfig.getStepVoltage()),
                Seconds.of(sysIdConfig.getQuasistaticTimeoutSeconds() + sysIdConfig.getDynamicTimeoutSeconds()
                        + sysIdConfig.getDelaySeconds() * 3));
        SysIdRoutine routine = SwerveDriveTest.setDriveSysIdRoutine(
                routineConfig, subsystem, drive, sysIdConfig.getStepVoltage(), false);
        return SwerveDriveTest.generateSysIdCommand(
                routine, sysIdConfig.getDelaySeconds(),
                sysIdConfig.getQuasistaticTimeoutSeconds(),
                sysIdConfig.getDynamicTimeoutSeconds());
    }

    /**
     * Creates a SysId command that exercises all four steer (angle) motors simultaneously using the configured YAGSL
     * characterization routine. Timing is read from the drivebase SysId config.
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

        SysIdRoutineConfig sysIdConfig = subsystem.getConfig().sysId;
        SysIdRoutine.Config routineConfig = new SysIdRoutine.Config(
                Volts.per(edu.wpi.first.units.Units.Second).of(sysIdConfig.getRampRateVoltsPerSecond()),
                Volts.of(sysIdConfig.getStepVoltage()),
                Seconds.of(sysIdConfig.getQuasistaticTimeoutSeconds() + sysIdConfig.getDynamicTimeoutSeconds()
                        + sysIdConfig.getDelaySeconds() * 3));
        SysIdRoutine routine = SwerveDriveTest.setAngleSysIdRoutine(routineConfig, subsystem, drive);
        return SwerveDriveTest.generateSysIdCommand(
                routine, sysIdConfig.getDelaySeconds(),
                sysIdConfig.getQuasistaticTimeoutSeconds(),
                sysIdConfig.getDynamicTimeoutSeconds());
    }

    /**
     * Creates a SysId command that exercises a single drive motor identified by module index. All other drive motors
     * are held at zero voltage. The module is centered (angle set to 0 degrees) before the test begins.
     * <p>
     * Use the {@code MODULE_FRONT_LEFT}, {@code MODULE_FRONT_RIGHT}, {@code MODULE_BACK_LEFT}, and
     * {@code MODULE_BACK_RIGHT} constants for the module index.
     * </p>
     *
     * @param moduleIndex index of the swerve module (0 = front-left, 1 = front-right, 2 = back-left, 3 = back-right)
     * @return command that runs a full SysId sweep on the specified drive motor
     */
    public Command createDriveSysIdCommandForModule(int moduleIndex) {
        if (subsystem.isSubsystemDisabled()) {
            return Commands.print("Drive SysId skipped: drive base disabled.");
        }

        SwerveDrive drive = subsystem.getSwerveDrive();
        if (drive == null) {
            return Commands.print("Drive SysId skipped: swerve drive not configured.");
        }

        SwerveModule[] modules = drive.getModules();
        if (moduleIndex < 0 || moduleIndex >= modules.length) {
            return Commands.print("Drive SysId skipped: invalid module index " + moduleIndex + ".");
        }

        SwerveModule targetModule = modules[moduleIndex];
        String moduleName = MODULE_NAMES[moduleIndex];
        SysIdRoutineConfig sysIdConfig = subsystem.getConfig().sysId;

        SysIdRoutine.Config routineConfig = new SysIdRoutine.Config(
                Volts.per(edu.wpi.first.units.Units.Second).of(sysIdConfig.getRampRateVoltsPerSecond()),
                Volts.of(sysIdConfig.getStepVoltage()),
                Seconds.of(sysIdConfig.getQuasistaticTimeoutSeconds() + sysIdConfig.getDynamicTimeoutSeconds()
                        + sysIdConfig.getDelaySeconds() * 3));

        SysIdRoutine routine = new SysIdRoutine(routineConfig, new SysIdRoutine.Mechanism(
                (Voltage voltage) -> {
                    // Center the target module and apply voltage only to its drive motor.
                    if (!SwerveDriveTelemetry.isSimulation) {
                        targetModule.getAngleMotor().setReference(0, 0);
                        targetModule.getDriveMotor().setVoltage(voltage.in(Volts));
                        // Hold all other drive motors at zero.
                        for (int i = 0; i < modules.length; i++) {
                            if (i != moduleIndex) {
                                modules[i].getDriveMotor().setVoltage(0);
                            }
                        }
                    }
                },
                log -> SwerveDriveTest.logDriveMotorVoltage(targetModule, log),
                subsystem,
                "drive-" + moduleName));

        return SwerveDriveTest.generateSysIdCommand(
                routine, sysIdConfig.getDelaySeconds(),
                sysIdConfig.getQuasistaticTimeoutSeconds(),
                sysIdConfig.getDynamicTimeoutSeconds());
    }

    /**
     * Creates a SysId command that exercises a single angle (steer) motor identified by module index. All other angle
     * motors are held at zero voltage and all drive motors are held at zero. This isolates the angular response of a
     * single module for characterization.
     * <p>
     * Use the {@code MODULE_FRONT_LEFT}, {@code MODULE_FRONT_RIGHT}, {@code MODULE_BACK_LEFT}, and
     * {@code MODULE_BACK_RIGHT} constants for the module index.
     * </p>
     *
     * @param moduleIndex index of the swerve module (0 = front-left, 1 = front-right, 2 = back-left, 3 = back-right)
     * @return command that runs a full SysId sweep on the specified angle motor
     */
    public Command createAngleSysIdCommandForModule(int moduleIndex) {
        if (subsystem.isSubsystemDisabled()) {
            return Commands.print("Angle SysId skipped: drive base disabled.");
        }

        SwerveDrive drive = subsystem.getSwerveDrive();
        if (drive == null) {
            return Commands.print("Angle SysId skipped: swerve drive not configured.");
        }

        SwerveModule[] modules = drive.getModules();
        if (moduleIndex < 0 || moduleIndex >= modules.length) {
            return Commands.print("Angle SysId skipped: invalid module index " + moduleIndex + ".");
        }

        SwerveModule targetModule = modules[moduleIndex];
        String moduleName = MODULE_NAMES[moduleIndex];
        SysIdRoutineConfig sysIdConfig = subsystem.getConfig().sysId;

        SysIdRoutine.Config routineConfig = new SysIdRoutine.Config(
                Volts.per(edu.wpi.first.units.Units.Second).of(sysIdConfig.getRampRateVoltsPerSecond()),
                Volts.of(sysIdConfig.getStepVoltage()),
                Seconds.of(sysIdConfig.getQuasistaticTimeoutSeconds() + sysIdConfig.getDynamicTimeoutSeconds()
                        + sysIdConfig.getDelaySeconds() * 3));

        SysIdRoutine routine = new SysIdRoutine(routineConfig, new SysIdRoutine.Mechanism(
                (Voltage voltage) -> {
                    if (!SwerveDriveTelemetry.isSimulation) {
                        // Apply voltage only to the target angle motor.
                        targetModule.getAngleMotor().setVoltage(voltage.in(Volts));
                        // Hold all drive motors and other angle motors at zero.
                        for (int i = 0; i < modules.length; i++) {
                            modules[i].getDriveMotor().setVoltage(0);
                            if (i != moduleIndex) {
                                modules[i].getAngleMotor().setVoltage(0);
                            }
                        }
                    }
                },
                log -> SwerveDriveTest.logAngularMotorVoltage(targetModule, log),
                subsystem,
                "angle-" + moduleName));

        return SwerveDriveTest.generateSysIdCommand(
                routine, sysIdConfig.getDelaySeconds(),
                sysIdConfig.getQuasistaticTimeoutSeconds(),
                sysIdConfig.getDynamicTimeoutSeconds());
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
