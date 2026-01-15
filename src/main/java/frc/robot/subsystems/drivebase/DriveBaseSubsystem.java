package frc.robot.subsystems.drivebase;

import java.io.File;
import java.util.Optional;

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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.DriveBaseSubsystemConfig;
import frc.robot.subsystems.AbstractSubsystem;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Provides a minimal API surface for commanding the robot drive base. It encapsulates the swerve hardware and exposes semantic operations for teleop
 * driving and pose targeting so commands can focus on higher-level logic.
 */
public class DriveBaseSubsystem extends AbstractSubsystem<DriveBaseSubsystemConfig> {

    private final Translation2d               centerOfRotationMeters = new Translation2d();

    private final PIDController               xController;

    private final PIDController               yController;

    private final ProfiledPIDController       thetaController;

    private final HolonomicDriveController    holonomicController;

    private final DriveBaseIO                 io;

    private final DriveBaseIOInputsAutoLogged inputs                 = new DriveBaseIOInputsAutoLogged();

    private final Field2d                     fieldDisplay           = new Field2d();

    private SwerveDrive                       swerveDrive;

    private SwerveController                  swerveController;

    private Optional<Pose2d>                  targetPose             = Optional.empty();

    public DriveBaseSubsystem(DriveBaseSubsystemConfig config) {
        super(config);

        this.xController         = createTranslationController();
        this.yController         = createTranslationController();
        this.thetaController     = createThetaController();
        this.holonomicController = new HolonomicDriveController(xController, yController, thetaController);
        this.holonomicController.setTolerance(
                new Pose2d(
                        new Translation2d(config.getTranslationToleranceMeters(), config.getTranslationToleranceMeters()),
                        Rotation2d.fromRadians(config.getRotationToleranceRadians())));

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
        if (isSubsystemDisabled() || swerveDrive == null) {
            return;
        }

        try {
            swerveDrive.updateOdometry();
        } catch (NullPointerException npe) {
            log.warning("Odometry update skipped because telemetry arrays are not initialized yet.");
        }

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
     * Drives the robot using robot-relative chassis speeds (forward, left, CCW rotation).
     *
     * @param vxMetersPerSecond     Forward velocity request.
     * @param vyMetersPerSecond     Leftward velocity request.
     * @param omegaRadiansPerSecond Counter-clockwise rotational rate.
     */
    public void driveRobotRelative(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        requestDrive(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, false);
    }

    /**
     * Drives the robot using field-relative chassis speeds.
     *
     * @param vxMetersPerSecond     Field-forward velocity request.
     * @param vyMetersPerSecond     Field-left velocity request.
     * @param omegaRadiansPerSecond Counter-clockwise rotational rate.
     */
    public void driveFieldRelative(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        requestDrive(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, true);
    }

    /**
     * Convenience overload to drive directly with {@link ChassisSpeeds} in robot space.
     *
     * @param speeds Robot-relative chassis speeds.
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        driveRobotRelative(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    /**
     * Convenience overload to drive directly with {@link ChassisSpeeds} in field space.
     *
     * @param speeds Field-relative chassis speeds.
     */
    public void driveFieldRelative(ChassisSpeeds speeds) {
        driveFieldRelative(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    /**
     * Drives field-relative while maintaining or chasing a desired heading using the YAGSL heading controller. This mirrors the example behavior
     * where a heading setpoint is converted into an angular velocity before commanding the modules.
     *
     * @param vxMetersPerSecond Field-forward velocity request.
     * @param vyMetersPerSecond Field-left velocity request.
     * @param targetHeading     Desired field heading to hold or reach.
     */
    public void driveFieldRelativeHeading(double vxMetersPerSecond, double vyMetersPerSecond, Rotation2d targetHeading) {
        if (isSubsystemDisabled()) {
            return;
        }

        if (swerveDrive == null || swerveController == null) {
            log.warning("Drive request ignored because the swerve drive is not available.");
            return;
        }

        Translation2d translation = clampTranslation(new Translation2d(vxMetersPerSecond, vyMetersPerSecond));
        double        omega       = clampRotation(
                swerveController.headingCalculate(targetHeading.getRadians(), getPose().getRotation().getRadians()));

        swerveDrive.drive(translation, omega, true, false, centerOfRotationMeters);
        Logger.recordOutput("DriveBase/RequestedHeading", targetHeading);
    }

    /**
     * Stops all motion and locks the modules in their current orientation.
     */
    public void stop() {
        if (isSubsystemDisabled()) {
            return;
        }

        requestDrive(0.0, 0.0, 0.0, false);
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

        Logger.recordOutput("DriveBase/RequestedSpeeds", speeds);
        driveRobotRelative(speeds);
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

        return translationError <= config.getTranslationToleranceMeters()
                && rotationError <= config.getRotationToleranceRadians();
    }

    private PIDController createTranslationController() {
        PIDController controller = new PIDController(config.getTranslationKp(), config.getTranslationKi(), config.getTranslationKd());
        controller.setTolerance(config.getTranslationToleranceMeters());
        controller.setIntegratorRange(
                -config.getMaximumLinearSpeedMetersPerSecond(),
                config.getMaximumLinearSpeedMetersPerSecond());
        return controller;
    }

    private ProfiledPIDController createThetaController() {
        ProfiledPIDController controller = new ProfiledPIDController(
                config.getRotationKp(),
                config.getRotationKi(),
                config.getRotationKd(),
                new TrapezoidProfile.Constraints(
                        config.getMaximumAngularSpeedRadiansPerSecond(),
                        config.getMaximumAngularAccelerationRadiansPerSecondSquared()));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(config.getRotationToleranceRadians());
        return controller;
    }

    private void configureSwerveDrive() {
        try {
            File configDirectory = new File(Filesystem.getDeployDirectory(), config.getSwerveDirectory());
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;

            swerveDrive                    = new SwerveParser(configDirectory).createSwerveDrive(config.getMaximumLinearSpeedMetersPerSecond());

            if (RobotBase.isSimulation()) {
                SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
                swerveDrive.setHeadingCorrection(false);
                swerveDrive.setCosineCompensator(false);
            }

            swerveController = swerveDrive.swerveController;
            swerveController.thetaController.setTolerance(config.getRotationToleranceRadians(), 0.1);
            swerveController.thetaController.setPID(config.getHeadingKp(), config.getHeadingKi(), config.getHeadingKd());

            log.info("Drive base configured from " + configDirectory.getAbsolutePath());
        } catch (Throwable e) {
            enabled = false;
            log.error("Failed to configure swerve drive: " + e.getMessage());
            e.printStackTrace();
        }
    }

    private void requestDrive(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond, boolean fieldRelative) {
        if (isSubsystemDisabled()) {
            return;
        }

        if (swerveDrive == null) {
            log.warning("Drive request ignored because the swerve drive is not available.");
            return;
        }

        Translation2d translation = clampTranslation(new Translation2d(vxMetersPerSecond, vyMetersPerSecond));
        double        rotation    = clampRotation(omegaRadiansPerSecond);

        swerveDrive.drive(translation, rotation, fieldRelative, false, centerOfRotationMeters);
    }

    private Translation2d clampTranslation(Translation2d requestedVelocity) {
        double magnitude = requestedVelocity.getNorm();
        double maxSpeed  = config.getMaximumLinearSpeedMetersPerSecond();

        if (magnitude == 0.0 || magnitude <= maxSpeed) {
            return requestedVelocity;
        }

        double scale = maxSpeed / magnitude;
        return requestedVelocity.times(scale);
    }

    private double clampRotation(double omegaRadiansPerSecond) {
        double maxRotationSpeed = config.getMaximumAngularSpeedRadiansPerSecond();
        return MathUtil.clamp(omegaRadiansPerSecond, -maxRotationSpeed, maxRotationSpeed);
    }
}
