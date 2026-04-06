package frc.robot.subsystems.drivebase.commands;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.shared.bindings.DpadTargetConfig;
import frc.robot.shared.bindings.DriverControllerConfig;
import frc.robot.shared.bindings.TrenchZoneConfig;
import frc.robot.shared.commands.AbstractSubsystemCommandFactory;
import frc.robot.shared.config.RobotEnvironment;
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
    public static final int       MODULE_FRONT_LEFT  = 0;

    /** Module index for the front-right swerve module in the YAGSL modules array. */
    public static final int       MODULE_FRONT_RIGHT = 1;

    /** Module index for the back-left swerve module in the YAGSL modules array. */
    public static final int       MODULE_BACK_LEFT   = 2;

    /** Module index for the back-right swerve module in the YAGSL modules array. */
    public static final int       MODULE_BACK_RIGHT  = 3;

    /** Human-readable names for each module index, matching the YAGSL swervedrive.json ordering. */
    private static final String[] MODULE_NAMES       = { "FrontLeft", "FrontRight", "BackLeft", "BackRight" };

    /**
     * Creates a factory that produces commands operating on the provided drive base subsystem.
     *
     * @param subsystem drive base subsystem instance to be shared by generated commands
     */
    public DriveBaseSubsystemCommandFactory(DriveBaseSubsystem subsystem) {
        super(subsystem);
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
     * Creates a SysId command that exercises all four drive motors simultaneously using the configured YAGSL characterization routine. Timing is read
     * from the drivebase SysId config.
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

        SysIdRoutineConfig  sysIdConfig   = subsystem.getConfig().sysId;
        SysIdRoutine.Config routineConfig = new SysIdRoutine.Config(
                Volts.per(edu.wpi.first.units.Units.Second).of(sysIdConfig.getRampRateVoltsPerSecond()),
                Volts.of(sysIdConfig.getStepVoltage()),
                Seconds.of(sysIdConfig.getQuasistaticTimeoutSeconds() + sysIdConfig.getDynamicTimeoutSeconds()
                        + sysIdConfig.getDelaySeconds() * 3));
        SysIdRoutine        routine       = SwerveDriveTest.setDriveSysIdRoutine(
                routineConfig, subsystem, drive, sysIdConfig.getStepVoltage(), false);
        return SwerveDriveTest.generateSysIdCommand(
                routine, sysIdConfig.getDelaySeconds(),
                sysIdConfig.getQuasistaticTimeoutSeconds(),
                sysIdConfig.getDynamicTimeoutSeconds());
    }

    /**
     * Creates a SysId command that exercises all four steer (angle) motors simultaneously using the configured YAGSL characterization routine. Timing
     * is read from the drivebase SysId config.
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

        SysIdRoutineConfig  sysIdConfig   = subsystem.getConfig().sysId;
        SysIdRoutine.Config routineConfig = new SysIdRoutine.Config(
                Volts.per(edu.wpi.first.units.Units.Second).of(sysIdConfig.getRampRateVoltsPerSecond()),
                Volts.of(sysIdConfig.getStepVoltage()),
                Seconds.of(sysIdConfig.getQuasistaticTimeoutSeconds() + sysIdConfig.getDynamicTimeoutSeconds()
                        + sysIdConfig.getDelaySeconds() * 3));
        SysIdRoutine        routine       = SwerveDriveTest.setAngleSysIdRoutine(routineConfig, subsystem, drive);
        return SwerveDriveTest.generateSysIdCommand(
                routine, sysIdConfig.getDelaySeconds(),
                sysIdConfig.getQuasistaticTimeoutSeconds(),
                sysIdConfig.getDynamicTimeoutSeconds());
    }

    /**
     * Creates a SysId command that exercises a single drive motor identified by module index. All other drive motors are held at zero voltage. The
     * module is centered (angle set to 0 degrees) before the test begins.
     * <p>
     * Use the {@code MODULE_FRONT_LEFT}, {@code MODULE_FRONT_RIGHT}, {@code MODULE_BACK_LEFT}, and {@code MODULE_BACK_RIGHT} constants for the module
     * index.
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

        SwerveModule        targetModule  = modules[moduleIndex];
        String              moduleName    = MODULE_NAMES[moduleIndex];
        SysIdRoutineConfig  sysIdConfig   = subsystem.getConfig().sysId;

        SysIdRoutine.Config routineConfig = new SysIdRoutine.Config(
                Volts.per(edu.wpi.first.units.Units.Second).of(sysIdConfig.getRampRateVoltsPerSecond()),
                Volts.of(sysIdConfig.getStepVoltage()),
                Seconds.of(sysIdConfig.getQuasistaticTimeoutSeconds() + sysIdConfig.getDynamicTimeoutSeconds()
                        + sysIdConfig.getDelaySeconds() * 3));

        SysIdRoutine        routine       = new SysIdRoutine(routineConfig, new SysIdRoutine.Mechanism(
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
     * Creates a SysId command that exercises a single angle (steer) motor identified by module index. All other angle motors are held at zero voltage
     * and all drive motors are held at zero. This isolates the angular response of a single module for characterization.
     * <p>
     * Use the {@code MODULE_FRONT_LEFT}, {@code MODULE_FRONT_RIGHT}, {@code MODULE_BACK_LEFT}, and {@code MODULE_BACK_RIGHT} constants for the module
     * index.
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

        SwerveModule        targetModule  = modules[moduleIndex];
        String              moduleName    = MODULE_NAMES[moduleIndex];
        SysIdRoutineConfig  sysIdConfig   = subsystem.getConfig().sysId;

        SysIdRoutine.Config routineConfig = new SysIdRoutine.Config(
                Volts.per(edu.wpi.first.units.Units.Second).of(sysIdConfig.getRampRateVoltsPerSecond()),
                Volts.of(sysIdConfig.getStepVoltage()),
                Seconds.of(sysIdConfig.getQuasistaticTimeoutSeconds() + sysIdConfig.getDynamicTimeoutSeconds()
                        + sysIdConfig.getDelaySeconds() * 3));

        SysIdRoutine        routine       = new SysIdRoutine(routineConfig, new SysIdRoutine.Mechanism(
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
            double                  currentRadians      = subsystem.getFusedPose().getRotation().getRadians();
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

            double                  currentRadians      = subsystem.getFusedPose().getRotation().getRadians();
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

    /**
     * Builds a command that pathfinds to a d-pad target with trench zone awareness.
     * <p>
     * The target is stored in blue-alliance coordinates and flipped at runtime for the red alliance. When the straight-line path from the robot's
     * current position to the target crosses a configured trench zone, intermediate waypoints are inserted at the zone entry and exit so the robot
     * drives through the trench with the correct heading. The robot begins translating and rotating simultaneously toward the trench entry to
     * eliminate the delay of a stop-and-rotate step. When no trench zone is crossed, a single pathfind command drives directly to the target.
     * </p>
     *
     * @param targetConfig config holding the target pose for this d-pad direction (blue-alliance coordinates)
     * @param driverConfig shared driver controller config holding pathfinding constraints and trench zone definitions
     * @return command that pathfinds to the target pose with trench zone handling, or a no-op if the subsystem is disabled
     */
    public Command createDpadPathfindCommand(
            DpadTargetConfig targetConfig,
            DriverControllerConfig driverConfig) {
        if (subsystem.isSubsystemDisabled()) {
            return Commands.print("Dpad pathfind skipped: drive base disabled.");
        }

        return Commands.deferredProxy(() -> {
            Pose2d           bluePose    = targetConfig.toPose2d();

            // Flip the target for the red alliance.
            Alliance         alliance    = RobotEnvironment.getAlliance().orElse(Alliance.Blue);
            Pose2d           targetPose  = alliance == Alliance.Red ? FlippingUtil.flipFieldPose(bluePose) : bluePose;

            PathConstraints  constraints = new PathConstraints(
                    driverConfig.getDpadMaxVelocityMetersPerSecond(),
                    driverConfig.getDpadMaxAccelerationMetersPerSecondSquared(),
                    Units.degreesToRadians(driverConfig.getDpadMaxAngularVelocityDegreesPerSecond()),
                    Units.degreesToRadians(driverConfig.getDpadMaxAngularAccelerationDegreesPerSecondSquared()));

            // Check if the path crosses a trench zone.
            Pose2d           currentPose = subsystem.getFusedPose();
            TrenchZoneConfig trenchZone  = driverConfig.findIntersectingTrenchZone(currentPose, targetPose);

            if (trenchZone != null) {
                Pose2d entryPose     = trenchZone.computeEntryWaypoint(currentPose, targetPose);
                Pose2d exitPose      = trenchZone.computeExitWaypoint(currentPose, targetPose);
                double trenchHeading = entryPose.getRotation().getRadians();

                // Drive toward the entry while simultaneously aligning to the trench heading,
                // then traverse the trench and pathfind to the final target.
                return Commands.sequence(
                        createDriveTowardPoseWithHeadingCommand(
                                entryPose, trenchHeading, driverConfig.getDpadMaxVelocityMetersPerSecond()),
                        createDriveStraightWithHeadingCommand(
                                exitPose, driverConfig.getDpadMaxVelocityMetersPerSecond()),
                        createPathfindToPoseCommand(targetPose, constraints));
            }

            return createPathfindToPoseCommand(targetPose, constraints);
        });
    }

    /**
     * Builds a field-relative manual driving command driven by the supplied control inputs.
     *
     * @param forwardMetersPerSecondSupplier supplier of forward (field +X) velocity in meters per second
     * @param leftMetersPerSecondSupplier    supplier of leftward (field +Y) velocity in meters per second
     * @param ccwRadiansPerSecondSupplier    supplier of counter-clockwise angular velocity in radians per second
     * @return command that continues driving until interrupted
     */
    private MoveFieldManualCommand createMoveManualCommand(
            DoubleSupplier forwardMetersPerSecondSupplier,
            DoubleSupplier leftMetersPerSecondSupplier,
            DoubleSupplier ccwRadiansPerSecondSupplier) {
        return new MoveFieldManualCommand(subsystem, forwardMetersPerSecondSupplier, leftMetersPerSecondSupplier, ccwRadiansPerSecondSupplier);
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
    private MoveFieldManualWithHeadingCommand createMoveManualWithHeadingCommand(
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
     * Builds a command that rotates the robot in place to a target heading using the heading PID controller, then finishes.
     * <p>
     * The command drives with zero translation and PID-controlled omega until the heading error stays within the configured rotation tolerance for
     * 100 milliseconds. The debounce prevents the command from ending while the robot is still oscillating through the target. A 2-second safety
     * timeout prevents indefinite spinning if the PID cannot converge.
     * </p>
     *
     * @param targetHeadingRadians desired field-relative heading in radians (counter-clockwise positive)
     * @return command that rotates in place until the heading has settled within tolerance, or a no-op if the subsystem is disabled
     */
    private Command createRotateToHeadingCommand(double targetHeadingRadians) {
        if (subsystem.isSubsystemDisabled()) {
            return Commands.print("Rotate skipped: drive base disabled.");
        }

        double    normalizedTarget = MathUtil.angleModulus(targetHeadingRadians);

        // Require heading within tolerance for 100ms before declaring settled.
        Debouncer settledDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

        return Commands.runOnce(() -> {
            subsystem.resetHeadingController();
            settledDebouncer.calculate(false);
        })
                .andThen(Commands.run(
                        () -> subsystem.driveFieldRelativeWithHeading(0.0, 0.0, normalizedTarget),
                        subsystem))
                .until(() -> {
                    double currentRadians = subsystem.getFusedPose().getRotation().getRadians();
                    double error  = Math.abs(MathUtil.angleModulus(normalizedTarget - currentRadians));
                    return settledDebouncer.calculate(error < subsystem.getRotationToleranceRadians());
                })
                .withTimeout(2.0)
                .withName("RotateToHeading");
    }

    /**
     * Builds a command that drives toward a target position while simultaneously rotating to a desired heading.
     * <p>
     * Unlike the rotate-then-pathfind approach, this command begins translating and rotating at the same time, eliminating the stop-and-spin delay.
     * Field-relative velocity is computed toward the target position and {@code driveFieldRelativeWithHeading} locks the heading via PID. Speed is
     * proportionally reduced as the robot approaches the target to avoid overshooting.
     * </p>
     *
     * @param target                  target pose in field coordinates; position defines where to drive
     * @param targetHeadingRadians    desired field-relative heading in radians (counter-clockwise positive)
     * @param maxSpeedMetersPerSecond maximum translation speed in meters per second
     * @return command that drives to the target while rotating to the heading, or a no-op if the subsystem is disabled
     */
    private Command createDriveTowardPoseWithHeadingCommand(Pose2d target, double targetHeadingRadians, double maxSpeedMetersPerSecond) {
        if (subsystem.isSubsystemDisabled()) {
            return Commands.print("Drive-toward skipped: drive base disabled.");
        }

        double normalizedHeading       = MathUtil.angleModulus(targetHeadingRadians);
        double positionToleranceMeters = 0.3;

        return Commands.runOnce(() -> subsystem.resetHeadingController())
                .andThen(Commands.run(() -> {
                    Pose2d        current  = subsystem.getFusedPose();
                    Translation2d delta    = target.getTranslation().minus(current.getTranslation());
                    double        distance = delta.getNorm();

                    if (distance < 0.01) {
                        subsystem.driveFieldRelativeWithHeading(0.0, 0.0, normalizedHeading);
                        return;
                    }

                    // Squared ramp: decelerates aggressively in the final meter to avoid
                    // overshooting into walls. Floor of 0.4 m/s keeps the robot creeping
                    // rather than coasting on inertia.
                    double speed = Math.min(maxSpeedMetersPerSecond, Math.max(0.4, distance * distance * 4.0));
                    double vx    = speed * delta.getX() / distance;
                    double vy    = speed * delta.getY() / distance;

                    subsystem.driveFieldRelativeWithHeading(vx, vy, normalizedHeading);
                }, subsystem))
                .until(() -> {
                    double distance = target.getTranslation().getDistance(
                            subsystem.getFusedPose().getTranslation());
                    return distance < positionToleranceMeters;
                })
                .withTimeout(5.0)
                .withName("DriveTowardPoseWithHeading");
    }

    /**
     * Builds a command that drives toward a target position while maintaining a fixed heading using PID control.
     * <p>
     * Used for trench traversal where the robot must hold a specific heading while translating through a narrow passage. The command computes
     * field-relative velocity toward the target position and uses {@code driveFieldRelativeWithHeading} to lock the heading. Speed is proportionally
     * reduced as the robot approaches the target to avoid overshooting.
     * </p>
     *
     * @param target                  target pose in field coordinates; rotation defines the heading to maintain
     * @param maxSpeedMetersPerSecond maximum translation speed in meters per second
     * @return command that drives to the target position while maintaining heading, or a no-op if the subsystem is disabled
     */
    private Command createDriveStraightWithHeadingCommand(Pose2d target, double maxSpeedMetersPerSecond) {
        if (subsystem.isSubsystemDisabled()) {
            return Commands.print("Drive-straight skipped: drive base disabled.");
        }

        double headingRadians          = target.getRotation().getRadians();
        double positionToleranceMeters = 0.3;

        return Commands.run(() -> {
            Pose2d        current  = subsystem.getFusedPose();
            Translation2d delta    = target.getTranslation().minus(current.getTranslation());
            double        distance = delta.getNorm();

            if (distance < 0.01) {
                subsystem.driveFieldRelativeWithHeading(0.0, 0.0, headingRadians);
                return;
            }

            // Squared ramp: decelerates aggressively in the final meter to avoid
            // overshooting into walls. Floor of 0.4 m/s keeps the robot creeping
            // rather than coasting on inertia.
            double speed = Math.min(maxSpeedMetersPerSecond, Math.max(0.4, distance * distance * 4.0));
            double vx    = speed * delta.getX() / distance;
            double vy    = speed * delta.getY() / distance;

            subsystem.driveFieldRelativeWithHeading(vx, vy, headingRadians);
        }, subsystem)
                .until(() -> {
                    double distance = target.getTranslation().getDistance(
                            subsystem.getFusedPose().getTranslation());
                    return distance < positionToleranceMeters;
                })
                .withTimeout(5.0)
                .withName("DriveStraightWithHeading");
    }

    /**
     * Builds a command that pathfinds from the robot's current pose to the given target pose using PathPlanner's AD* pathfinder.
     * <p>
     * The command respects obstacles defined in the PathPlanner navgrid and ends at zero velocity when the target is reached. Use this for teleop
     * d-pad pathfinding where the driver holds a button to drive to a pre-configured field position.
     * </p>
     *
     * @param targetPose  field pose to pathfind to (already alliance-corrected by the caller)
     * @param constraints velocity and acceleration limits for the pathfinding trajectory
     * @return command that pathfinds to the target pose and stops, or a no-op if the subsystem is disabled
     */
    private Command createPathfindToPoseCommand(Pose2d targetPose, PathConstraints constraints) {
        if (subsystem.isSubsystemDisabled()) {
            return Commands.print("Pathfind skipped: drive base disabled.");
        }

        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }
}
