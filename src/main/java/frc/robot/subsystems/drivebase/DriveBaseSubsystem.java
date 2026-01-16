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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
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

    private static final double               JOYSTICK_DEADBAND       = 0.08;

    private final Translation2d               centerOfRotationMeters  = new Translation2d();

    private final PIDController               xController;

    private final PIDController               yController;

    private final ProfiledPIDController       thetaController;

    private final HolonomicDriveController    holonomicController;

    private final DriveBaseIO                 io;

    private final DriveBaseIOInputsAutoLogged inputs                  = new DriveBaseIOInputsAutoLogged();

    private final Field2d                     fieldDisplay            = new Field2d();

    private SwerveDrive                       swerveDrive;

    private SwerveController                  swerveController;

    private boolean                           odometryWarningEmitted  = false;

    private double                            lastOdometryWarningTime = -1.0;

    private Optional<Pose2d>                  targetPose              = Optional.empty();

    public DriveBaseSubsystem(DriveBaseSubsystemConfig config) {
        super(config);

        this.xController         = createTranslationController();
        this.yController         = createTranslationController();
        this.thetaController     = createThetaController();
        this.holonomicController = new HolonomicDriveController(xController, yController, thetaController);
        this.holonomicController.setTolerance(
                new Pose2d(
                        new Translation2d(
                                config.getTranslationToleranceMeters().get(),
                                config.getTranslationToleranceMeters().get()),
                        Rotation2d.fromRadians(config.getRotationToleranceRadians().get())));

        if (isSubsystemDisabled()) {
            log.verbose("DriveBaseSubsystem disabled; skipping hardware init.");
            this.io = inputs -> {
            };
            return;
        }

        configureSwerveDrive();
        this.io = swerveDrive != null ? new DriveBaseIOYagsl(swerveDrive) : inputs -> {
        };

        SmartDashboard.putData("Field", fieldDisplay);
    }

    /**
     * Called every robot tick to publish telemetry.
     */
    @Override
    public void periodic() {
        if (!DriverStation.isFMSAttached()) {
            refreshTunables();
        }

        if (isSubsystemDisabled() || swerveDrive == null) {
            return;
        }

        updateOdometrySafely();

        io.updateInputs(inputs);
        Logger.processInputs("DriveBase", inputs);
        Logger.recordOutput("Odometry/Robot", getPose());
        Logger.recordOutput("SwerveStates/Measured", inputs.moduleStates);
        Logger.recordOutput("SwerveStates/Target", inputs.moduleTargets);
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
        double        rawForward        = -forwardAxis;
        double        rawLeft           = -leftAxis;

        double        deadbandedForward = MathUtil.applyDeadband(rawForward, JOYSTICK_DEADBAND);
        double        deadbandedLeft    = MathUtil.applyDeadband(rawLeft, JOYSTICK_DEADBAND);

        Translation2d rawVector         = new Translation2d(deadbandedForward, deadbandedLeft);
        double        translationScale  = config.getTranslationScale().get();
        Translation2d scaledVector      = SwerveMath.scaleTranslation(rawVector, translationScale);
        Translation2d commandedSpeeds   = new Translation2d(
                scaledVector.getX() * config.getMaximumLinearSpeedMetersPerSecond().get(),
                scaledVector.getY() * config.getMaximumLinearSpeedMetersPerSecond().get());

        log.recordOutput("DriverInputs/forward/raw", rawForward);
        log.recordOutput("DriverInputs/forward/deadbanded", deadbandedForward);
        log.recordOutput("DriverInputs/left/raw", rawLeft);
        log.recordOutput("DriverInputs/left/deadbanded", deadbandedLeft);
        log.recordOutput("DriverInputs/translation/scale", translationScale);
        log.recordOutput("DriverInputs/translation/scaledX", scaledVector.getX());
        log.recordOutput("DriverInputs/translation/scaledY", scaledVector.getY());
        log.recordOutput("DriverInputs/translation/commandedX", commandedSpeeds.getX());
        log.recordOutput("DriverInputs/translation/commandedY", commandedSpeeds.getY());

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
     *
     * @param omegaAxis rotation stick value (positive for counter-clockwise)
     * @return angular velocity request in radians per second
     */
    public double mapDriverOmega(double omegaAxis) {
        double processed        = MathUtil.applyDeadband(-omegaAxis, JOYSTICK_DEADBAND);
        double radiansPerSecond = processed * config.getMaximumAngularSpeedRadiansPerSecond().get();

        log.recordOutput("DriverInputs/omega/raw", omegaAxis);
        log.recordOutput("DriverInputs/omega/deadbanded", processed);
        log.recordOutput("DriverInputs/omega/radiansPerSecond", radiansPerSecond);

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
     * Overrides the translation scale source so values can be fed from SmartDashboard or another tuning path.
     *
     * @param scaleSupplier supplier returning a 0–1 translation scale; falls back to config if null
     */
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
     * Drives toward the target pose using the holonomic controller.
     */
    public void seekTarget() {
        if (isSubsystemDisabled()) {
            return;
        }

        if (targetPose.isEmpty()) {
            log.warning("seekTarget called without a configured target pose.");
            return;
        }

        if (swerveDrive == null) {
            log.warning("Swerve drive not configured; cannot seek target pose.");
            return;
        }

        if (atTarget()) {
            stop();
            return;
        }

        Pose2d        goal        = targetPose.get();
        Pose2d        currentPose = getPose();
        ChassisSpeeds speeds      = holonomicController.calculate(currentPose, goal, 0.0, goal.getRotation());
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, currentPose.getRotation());

        Logger.recordOutput("DriveBase/RequestedSpeeds", speeds);
        Logger.recordOutput("DriveBase/RequestedFieldSpeeds", fieldSpeeds);
        driveFieldRelative(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond, fieldSpeeds.omegaRadiansPerSecond);
    }

    /**
     * Reports whether the robot pose is within the configured tolerances of the active target.
     *
     * @return True when the robot is at the requested pose or no target exists.
     */
    public boolean atTarget() {
        if (targetPose.isEmpty()) {
            return true;
        }

        Pose2d goal             = targetPose.get();
        Pose2d currentPose      = getPose();
        double translationError = currentPose.getTranslation().getDistance(goal.getTranslation());
        double rotationError    = Math.abs(MathUtil.angleModulus(currentPose.getRotation().minus(goal.getRotation()).getRadians()));

        return translationError <= config.getTranslationToleranceMeters().get()
                && rotationError <= config.getRotationToleranceRadians().get();
    }

    private PIDController createTranslationController() {
        PIDController controller = new PIDController(
                config.getTranslationKp().get(),
                config.getTranslationKi().get(),
                config.getTranslationKd().get());
        controller.setTolerance(config.getTranslationToleranceMeters().get());
        controller.setIntegratorRange(
                -config.getMaximumLinearSpeedMetersPerSecond().get(),
                config.getMaximumLinearSpeedMetersPerSecond().get());
        return controller;
    }

    private ProfiledPIDController createThetaController() {
        ProfiledPIDController controller = new ProfiledPIDController(
                config.getRotationKp().get(),
                config.getRotationKi().get(),
                config.getRotationKd().get(),
                new TrapezoidProfile.Constraints(
                        config.getMaximumAngularSpeedRadiansPerSecond().get(),
                        config.getMaximumAngularAccelerationRadiansPerSecondSquared().get()));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(config.getRotationToleranceRadians().get());
        return controller;
    }

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

    private void configureSwerveDrive() {
        try {
            File configDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;

            swerveDrive                    = new SwerveParser(configDirectory)
                    .createSwerveDrive(config.getMaximumLinearSpeedMetersPerSecond().get());

            if (RobotBase.isSimulation()) {
                swerveDrive.setHeadingCorrection(false);
                swerveDrive.setCosineCompensator(false);
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

        swerveDrive.drive(translation, rotation, true, false, centerOfRotationMeters);
    }

    private Translation2d clampTranslation(Translation2d requestedVelocity) {
        double magnitude = requestedVelocity.getNorm();
        double maxSpeed  = config.getMaximumLinearSpeedMetersPerSecond().get();

        if (magnitude == 0.0 || magnitude <= maxSpeed) {
            return requestedVelocity;
        }

        double scale = maxSpeed / magnitude;
        return requestedVelocity.times(scale);
    }

    private double clampRotation(double omegaRadiansPerSecond) {
        double maxRotationSpeed = config.getMaximumAngularSpeedRadiansPerSecond().get();
        return MathUtil.clamp(omegaRadiansPerSecond, -maxRotationSpeed, maxRotationSpeed);
    }

    private void updateOdometrySafely() {
        if (swerveDrive == null) {
            return;
        }

        try {
            swerveDrive.updateOdometry();
            if (odometryWarningEmitted) {
                log.info("Odometry telemetry initialized; updates resumed.");
                odometryWarningEmitted = false;
            }
        } catch (NullPointerException npe) {
            emitOdometryWarningOncePerSecond();
        }
    }

    private void emitOdometryWarningOncePerSecond() {
        double now = Timer.getFPGATimestamp();
        if (!odometryWarningEmitted || now - lastOdometryWarningTime >= 1.0) {
            log.warning("Odometry update skipped because telemetry arrays are not initialized yet.");
            odometryWarningEmitted  = true;
            lastOdometryWarningTime = now;
        }
    }
}
