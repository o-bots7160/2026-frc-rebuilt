package frc.robot.subsystems.drivebase;

import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.shared.subsystems.AbstractSubsystem;
import frc.robot.subsystems.drivebase.config.DriveBaseSubsystemConfig;
import frc.robot.subsystems.drivebase.io.DriveBaseIO;
import frc.robot.subsystems.drivebase.io.DriveBaseIOInputsAutoLogged;
import frc.robot.subsystems.drivebase.io.DriveBaseIOYagsl;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Provides a minimal API surface for commanding the robot drive base. It encapsulates the swerve hardware and exposes semantic operations for teleop
 * driving and pose targeting so commands can focus on higher-level logic.
 */
public class DriveBaseSubsystem extends AbstractSubsystem<DriveBaseSubsystemConfig> {

    /**
     * Deadband size for joystick inputs.
     * <p>
     * Values inside this range are treated as zero to ignore tiny stick noise.
     * </p>
     */
    private static final double               JOYSTICK_DEADBAND      = 0.08;

    private final Translation2d               centerOfRotationMeters = new Translation2d();

    /**
     * PID controller for X (forward/back) translation.
     * <p>
     * PID (Proportional, Integral, Derivative) corrects position error over time.
     * </p>
     */
    private final PIDController               xController;

    /**
     * PID controller for Y (left/right) translation.
     * <p>
     * PID (Proportional, Integral, Derivative) corrects position error over time.
     * </p>
     */
    private final PIDController               yController;

    /**
     * Profiled PID controller for theta (robot heading angle) control.
     * <p>
     * Theta is the robot's rotation around the vertical axis, measured in radians.
     * </p>
     */
    private final ProfiledPIDController       thetaController;

    /**
     * Holonomic drive controller that combines X, Y, and theta control.
     * <p>
     * Holonomic means the robot can translate in any direction while rotating.
     * </p>
     */
    private final HolonomicDriveController    holonomicController;

    private final DriveBaseIO                 io;

    private final DriveBaseIOInputsAutoLogged inputs                 = new DriveBaseIOInputsAutoLogged();

    private final Field2d                     fieldDisplay           = new Field2d();

    /**
     * YAGSL swerve drive instance that handles kinematics, odometry, and module commands.
     * <p>
     * Swerve means each wheel can steer and drive, allowing the robot to move in any direction while rotating.
     * </p>
     */
    private SwerveDrive                       swerveDrive;

    /**
     * High-level swerve controller used for heading and motion control helpers.
     * <p>
     * Uses the swerve model to turn chassis speed requests into wheel states.
     * </p>
     */
    private SwerveController                  swerveController;

    private Optional<Pose2d>                  targetPose             = Optional.empty();

    private ChassisSpeeds                     lastRequestedSpeeds    = new ChassisSpeeds();

    private SwerveModuleState[]               lastRequestedStates    = new SwerveModuleState[0];

    public DriveBaseSubsystem(DriveBaseSubsystemConfig config) {
        super(config);

        // Build the translation and heading controllers used by holonomic path tracking.
        this.xController         = createTranslationController();
        this.yController         = createTranslationController();
        this.thetaController     = createThetaController();

        // Combine the axis controllers into a single holonomic controller.
        this.holonomicController = new HolonomicDriveController(xController, yController, thetaController);

        // Apply position/heading tolerances so the controller knows when it is “close enough.”
        this.holonomicController.setTolerance(
                new Pose2d(
                        new Translation2d(
                                config.getTranslationToleranceMeters().get(),
                                config.getTranslationToleranceMeters().get()),
                        Rotation2d.fromRadians(config.getRotationToleranceRadians().get())));

        // If the subsystem is disabled in config, skip all hardware setup.
        if (isSubsystemDisabled()) {
            log.verbose("DriveBaseSubsystem disabled; skipping hardware init.");
            this.io = inputs -> {
            };
            return;
        }

        // Initialize the swerve hardware from deploy configs and bind the IO layer.
        configureSwerveDrive();
        this.io = swerveDrive != null ? new DriveBaseIOYagsl(swerveDrive) : inputs -> {
        };

        // Publish the field visualization so dashboards can render the live robot pose.
        SmartDashboard.putData("Field", fieldDisplay);
    }

    /**
     * Called every robot tick to publish telemetry.
     */
    @Override
    public void periodic() {
        if (!isFMSAttached()) {
            refreshTunables();
        }

        if (isSubsystemDisabled() || swerveDrive == null) {
            return;
        }

        io.updateInputs(inputs);
        Logger.processInputs("DriveBase", inputs);
        Logger.recordOutput("Odometry/Robot", getPose());
        Logger.recordOutput("SwerveStates/Measured", inputs.moduleStates);
        Logger.recordOutput("SwerveStates/Target", lastRequestedStates);
        Logger.recordOutput("SwerveStates/CurrentStates", inputs.moduleStates);
        Logger.recordOutput("SwerveStates/DesiredStates", lastRequestedStates);
        Logger.recordOutput("SwerveChassisSpeeds/Measured", inputs.chassisSpeeds);
        Logger.recordOutput("SwerveChassisSpeeds/Desired", lastRequestedSpeeds);
        Logger.recordOutput("Swerve/RobotRotation", getPose().getRotation());
        Logger.recordOutput("DriveBase/HasTarget", targetPose.isPresent());
        targetPose.ifPresent(goal -> Logger.recordOutput("DriveBase/TargetPose", goal));

        fieldDisplay.setRobotPose(getPose());
    }

    /**
     * Returns the current estimated field pose of the robot.
     *
     * @return Current robot pose in meters and radians.
     */
    public Pose2d getPose() {
        if (swerveDrive == null) {
            return new Pose2d();
        }

        return swerveDrive.getPose();
    }

    /**
     * Resets odometry and gyro heading to the supplied pose.
     *
     * @param pose Desired pose for the robot to assume immediately.
     */
    public void resetPose(Pose2d pose) {
        if (isSubsystemDisabled() || swerveDrive == null) {
            return;
        }

        swerveDrive.resetOdometry(pose);
    }

    /**
     * Replaces the pose estimator state without changing odometry accumulators.
     *
     * @param pose Pose that should be reported going forward.
     */
    public void setPose(Pose2d pose) {
        if (isSubsystemDisabled() || swerveDrive == null) {
            return;
        }

        swerveDrive.swerveDrivePoseEstimator.resetPose(pose);
    }

    /**
     * Drives the robot using field-relative chassis speeds.
     *
     * @param vxMetersPerSecond     Field-forward velocity request.
     * @param vyMetersPerSecond     Field-left velocity request.
     * @param omegaRadiansPerSecond Counter-clockwise rotational rate.
     */
    public void driveFieldRelative(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        requestDrive(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }

    /**
     * Converts raw driver axes into scaled translation speeds in meters per second. Use this to keep joystick shaping, deadband, and telemetry in one
     * place.
     *
     * @param forwardAxis forward stick value (+X on the field)
     * @param leftAxis    left stick value (+Y on the field)
     * @return translation request in meters per second for field-relative driving
     */
    public Translation2d mapDriverTranslation(double forwardAxis, double leftAxis) {
        // Raw inputs: invert for driver preference and clamp to the joystick's legal range.
        double        rawForward        = MathUtil.clamp(-forwardAxis, -1.0, 1.0);
        double        rawLeft           = MathUtil.clamp(-leftAxis, -1.0, 1.0);

        // Deadband: ignore tiny stick noise near center so the robot stays still when hands are off.
        double        deadbandedForward = MathUtil.applyDeadband(rawForward, JOYSTICK_DEADBAND);
        double        deadbandedLeft    = MathUtil.applyDeadband(rawLeft, JOYSTICK_DEADBAND);

        // Vectorize: combine forward/left into a 2D translation request in joystick space.
        Translation2d rawVector         = new Translation2d(deadbandedForward, deadbandedLeft);

        // Scaling: limit how aggressive the driver translation feels (0 = no motion, 1 = full speed).
        double        translationScale  = MathUtil.clamp(config.getTranslationScale().get(), 0.0, 1.0);
        double        simulationScale   = 1.0;

        // Simulation: optionally tone down speeds and increase telemetry detail to help debugging.
        if (RobotBase.isSimulation()) {
            simulationScale  = MathUtil.clamp(config.getSimulationTranslationScale().get(), 0.0, 1.0);
            translationScale = translationScale * simulationScale;
        }

        // Shape and scale the translation while preserving direction (prevents diagonal overspeed).
        Translation2d scaledVector    = SwerveMath.scaleTranslation(rawVector, translationScale);

        // Convert the unitless vector into real robot speeds in meters per second.
        Translation2d commandedSpeeds = new Translation2d(
                scaledVector.getX() * config.getMaximumLinearSpeedMetersPerSecond().get(),
                scaledVector.getY() * config.getMaximumLinearSpeedMetersPerSecond().get());

        // Telemetry: record all driver input stages for tuning and debugging.
        log.recordOutput("DriverInputs/forward/raw", rawForward);
        log.recordOutput("DriverInputs/forward/deadbanded", deadbandedForward);
        log.recordOutput("DriverInputs/left/raw", rawLeft);
        log.recordOutput("DriverInputs/left/deadbanded", deadbandedLeft);
        log.recordOutput("DriverInputs/translation/scale", translationScale);
        log.recordOutput("DriverInputs/translation/simulationScale", simulationScale);
        log.recordOutput("DriverInputs/translation/scaledX", scaledVector.getX());
        log.recordOutput("DriverInputs/translation/scaledY", scaledVector.getY());
        log.recordOutput("DriverInputs/translation/commandedX", commandedSpeeds.getX());
        log.recordOutput("DriverInputs/translation/commandedY", commandedSpeeds.getY());

        // Return the final translation request in meters per second.
        return commandedSpeeds;
    }

    /**
     * Wraps {@link #mapDriverTranslation(double, double)} in a supplier so callers can hand the mapping function directly to commands.
     *
     * @param forwardAxisSupplier supplier of the forward stick value (+X on the field)
     * @param leftAxisSupplier    supplier of the left stick value (+Y on the field)
     * @return supplier that produces a field-relative translation request in meters per second
     */
    public Supplier<Translation2d> mapDriverTranslationSupplier(
            Supplier<Double> forwardAxisSupplier,
            Supplier<Double> leftAxisSupplier) {
        return () -> mapDriverTranslation(forwardAxisSupplier.get(), leftAxisSupplier.get());
    }

    /**
     * Converts a raw driver omega axis into a scaled angular speed in radians per second. Use this so all deadbanding and scaling stays in the
     * subsystem.
     * <p>
     * Omega is the robot's rotational velocity around the vertical axis, measured in radians per second.
     * </p>
     *
     * @param omegaAxis rotation stick value (positive for counter-clockwise)
     * @return angular velocity request in radians per second
     */
    public double mapDriverOmega(double omegaAxis) {
        // Raw input: invert for driver preference and clamp to the joystick's legal range.
        double rawAxis         = MathUtil.clamp(-omegaAxis, -1.0, 1.0);

        // Deadband: ignore tiny twist noise so the robot does not spin when the stick is centered.
        double processed       = MathUtil.applyDeadband(rawAxis, JOYSTICK_DEADBAND);
        double simulationScale = 1.0;

        // Simulation: optionally reduce angular speed for safer testing.
        if (RobotBase.isSimulation()) {
            simulationScale = MathUtil.clamp(config.getSimulationOmegaScale().get(), 0.0, 1.0);
        }

        // Convert the unitless stick value into real angular speed (radians per second).
        double radiansPerSecond = processed
                * simulationScale
                * config.getMaximumAngularSpeedRadiansPerSecond().get();

        // Telemetry: record all driver input stages for tuning and debugging.
        log.recordOutput("DriverInputs/omega/raw", rawAxis);
        log.recordOutput("DriverInputs/omega/deadbanded", processed);
        log.recordOutput("DriverInputs/omega/simulationScale", simulationScale);
        log.recordOutput("DriverInputs/omega/radiansPerSecond", radiansPerSecond);

        // Return the final angular velocity request in radians per second.
        return radiansPerSecond;
    }

    /**
     * Wraps {@link #mapDriverOmega(double)} in a supplier so callers can hand the mapping function directly to commands.
     *
     * @param omegaAxisSupplier supplier of the rotation stick value (positive for counter-clockwise)
     * @return supplier that produces a scaled angular velocity in radians per second
     */
    public Supplier<Double> mapDriverOmegaSupplier(Supplier<Double> omegaAxisSupplier) {
        return () -> mapDriverOmega(omegaAxisSupplier.get());
    }

    /**
     * Stops all motion and locks the modules in their current orientation.
     */
    public void stop() {
        if (isSubsystemDisabled()) {
            return;
        }

        requestDrive(0.0, 0.0, 0.0);
        if (swerveDrive != null) {
            swerveDrive.lockPose();
        }
    }

    /**
     * Sets a target pose for the holonomic controller to chase.
     *
     * @param pose Target pose expressed in field coordinates.
     */
    public void setTargetPose(Pose2d pose) {
        if (isSubsystemDisabled()) {
            return;
        }

        targetPose = Optional.ofNullable(pose);
        xController.reset();
        yController.reset();
        thetaController.reset(getPose().getRotation().getRadians());
    }

    /**
     * Sets only the translation portion of the target while preserving the desired heading.
     *
     * @param translation Translation to move toward.
     */
    public void setTargetTranslation(Translation2d translation) {
        Rotation2d rotation = targetPose.map(Pose2d::getRotation).orElseGet(() -> getPose().getRotation());
        setTargetPose(new Pose2d(translation, rotation));
    }

    /**
     * Sets only the rotation portion of the target while preserving the desired translation.
     *
     * @param rotation Desired field heading.
     */
    public void setTargetRotation(Rotation2d rotation) {
        Translation2d translation = targetPose.map(Pose2d::getTranslation).orElseGet(() -> getPose().getTranslation());
        setTargetPose(new Pose2d(translation, rotation));
    }

    /**
     * Clears any pending pose target so manual control can resume.
     */
    public void clearTargetPose() {
        targetPose = Optional.empty();
    }

    /**
     * Returns the currently configured target pose, if one exists.
     *
     * @return Optional pose of the active target.
     */
    public Optional<Pose2d> getTargetPose() {
        return targetPose;
    }

    /**
     * Provides access to the configured swerve drive for command factories and testing helpers.
     *
     * @return configured {@link SwerveDrive} instance, or {@code null} if initialization failed or subsystem is disabled
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Builds a PID controller for X/Y translation.
     * <p>
     * Use this for holonomic motion so the controller uses the current drivebase tuning.
     * </p>
     *
     * @return PID controller configured for translation in meters
     */
    private PIDController createTranslationController() {
        // PID gains: tuning for how strongly the robot corrects translation error.
        PIDController controller = new PIDController(
                config.getTranslationKp().get(),
                config.getTranslationKi().get(),
                config.getTranslationKd().get());

        // Tolerance: how close (in meters) we consider the target reached.
        controller.setTolerance(config.getTranslationToleranceMeters().get());

        // Integrator range: cap I-term to prevent windup beyond max linear speed.
        controller.setIntegratorRange(
                -config.getMaximumLinearSpeedMetersPerSecond().get(),
                config.getMaximumLinearSpeedMetersPerSecond().get());

        // Return the configured controller.
        return controller;
    }

    /**
     * Builds a profiled PID controller for heading control.
     * <p>
     * Theta is the robot's heading angle (rotation) around the vertical axis, measured in radians. The controller enforces angular speed and
     * acceleration limits so rotation stays smooth.
     * </p>
     *
     * @return profiled PID controller configured for radians
     */
    private ProfiledPIDController createThetaController() {
        // PID gains: tuning for how strongly the robot corrects heading error.
        ProfiledPIDController controller = new ProfiledPIDController(
                config.getRotationKp().get(),
                config.getRotationKi().get(),
                config.getRotationKd().get(),
                new TrapezoidProfile.Constraints(
                        config.getMaximumAngularSpeedRadiansPerSecond().get(),
                        config.getMaximumAngularAccelerationRadiansPerSecondSquared().get()));

        // Continuous input: treat -π and +π as the same angle so the controller picks the shortest turn.
        controller.enableContinuousInput(-Math.PI, Math.PI);

        // Tolerance: how close (in radians) we consider the heading target reached.
        controller.setTolerance(config.getRotationToleranceRadians().get());

        // Return the configured controller.
        return controller;
    }

    /**
     * Refreshes PID gains and tolerances from tunable config values.
     * <p>
     * Call this when tuning so any SmartDashboard changes take effect without a redeploy.
     * </p>
     */
    private void refreshTunables() {
        xController.setPID(
                config.getTranslationKp().get(),
                config.getTranslationKi().get(),
                config.getTranslationKd().get());
        xController.setTolerance(config.getTranslationToleranceMeters().get());
        xController.setIntegratorRange(
                -config.getMaximumLinearSpeedMetersPerSecond().get(),
                config.getMaximumLinearSpeedMetersPerSecond().get());

        yController.setPID(
                config.getTranslationKp().get(),
                config.getTranslationKi().get(),
                config.getTranslationKd().get());
        yController.setTolerance(config.getTranslationToleranceMeters().get());
        yController.setIntegratorRange(
                -config.getMaximumLinearSpeedMetersPerSecond().get(),
                config.getMaximumLinearSpeedMetersPerSecond().get());

        thetaController.setPID(
                config.getRotationKp().get(),
                config.getRotationKi().get(),
                config.getRotationKd().get());
        thetaController.setConstraints(new TrapezoidProfile.Constraints(
                config.getMaximumAngularSpeedRadiansPerSecond().get(),
                config.getMaximumAngularAccelerationRadiansPerSecondSquared().get()));
        thetaController.setTolerance(config.getRotationToleranceRadians().get());

        holonomicController.setTolerance(
                new Pose2d(
                        new Translation2d(
                                config.getTranslationToleranceMeters().get(),
                                config.getTranslationToleranceMeters().get()),
                        Rotation2d.fromRadians(config.getRotationToleranceRadians().get())));

        if (swerveController != null) {
            swerveController.thetaController.setTolerance(config.getRotationToleranceRadians().get(), 0.1);
            swerveController.thetaController.setPID(
                    config.getHeadingKp().get(),
                    config.getHeadingKi().get(),
                    config.getHeadingKd().get());
        }
    }

    /**
     * Loads the YAGSL swerve configuration and wires up the controllers.
     * <p>
     * This method configures telemetry and handles simulation-only settings.
     * </p>
     */
    private void configureSwerveDrive() {
        try {
            File configDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

            swerveDrive                    = new SwerveParser(configDirectory)
                    .createSwerveDrive(config.getMaximumLinearSpeedMetersPerSecond().get());

            if (isSimulation()) {
                swerveDrive.setHeadingCorrection(false);
                swerveDrive.setCosineCompensator(false);
                SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
            }

            swerveController = swerveDrive.swerveController;
            swerveController.thetaController.setTolerance(config.getRotationToleranceRadians().get(), 0.1);
            swerveController.thetaController.setPID(
                    config.getHeadingKp().get(),
                    config.getHeadingKi().get(),
                    config.getHeadingKd().get());

            log.info("Drive base configured from " + configDirectory.getAbsolutePath());
        } catch (Throwable e) {
            enabled = false;
            log.error("Failed to configure swerve drive: " + e.getMessage());
            e.printStackTrace();
        }
    }

    /**
     * Sends a field-relative drive request to the swerve drive.
     * <p>
     * Inputs are clamped to configured limits before commanding the hardware.
     * </p>
     *
     * @param vxMetersPerSecond     field-forward velocity in meters per second
     * @param vyMetersPerSecond     field-left velocity in meters per second
     * @param omegaRadiansPerSecond counter-clockwise rotation rate in radians per second
     */
    private void requestDrive(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        if (isSubsystemDisabled()) {
            return;
        }

        if (swerveDrive == null) {
            log.warning("Drive request ignored because the swerve drive is not available.");
            return;
        }

        Translation2d translation = clampTranslation(new Translation2d(vxMetersPerSecond, vyMetersPerSecond));
        double        rotation    = clampRotation(omegaRadiansPerSecond);

        lastRequestedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                swerveDrive.getOdometryHeading());
        lastRequestedStates = swerveDrive.toServeModuleStates(lastRequestedSpeeds, true);

        swerveDrive.drive(translation, rotation, true, false, centerOfRotationMeters);
    }

    /**
     * Limits translation commands to the maximum linear speed.
     * <p>
     * This keeps diagonal requests within the configured speed cap.
     * </p>
     *
     * @param requestedVelocity translation request in meters per second
     * @return possibly scaled translation request in meters per second
     */
    private Translation2d clampTranslation(Translation2d requestedVelocity) {
        double magnitude = requestedVelocity.getNorm();
        double maxSpeed  = config.getMaximumLinearSpeedMetersPerSecond().get();

        if (magnitude == 0.0 || magnitude <= maxSpeed) {
            return requestedVelocity;
        }

        double scale = maxSpeed / magnitude;
        return requestedVelocity.times(scale);
    }

    /**
     * Limits rotation commands to the maximum angular speed.
     * <p>
     * Use this to prevent the robot from spinning faster than the configured cap.
     * </p>
     *
     * @param omegaRadiansPerSecond desired rotation rate in radians per second
     * @return clamped rotation rate in radians per second
     */
    private double clampRotation(double omegaRadiansPerSecond) {
        double maxRotationSpeed = config.getMaximumAngularSpeedRadiansPerSecond().get();
        return MathUtil.clamp(omegaRadiansPerSecond, -maxRotationSpeed, maxRotationSpeed);
    }

}
