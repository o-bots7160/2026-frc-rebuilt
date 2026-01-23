package frc.robot.subsystems.drivebase;

import java.io.File;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

    private ChassisSpeeds                     lastRequestedSpeeds    = new ChassisSpeeds();

    private SwerveModuleState[]               lastRequestedStates    = new SwerveModuleState[0];

    public DriveBaseSubsystem(DriveBaseSubsystemConfig config) {
        super(config);

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
            // Only allow live tuning when we are not connected to the official field system.
            refreshTunables();
        }

        if (isSubsystemDisabled() || swerveDrive == null) {
            // No hardware to talk to, so skip the rest of the telemetry pipeline.
            return;
        }

        // Pull the latest sensor data from the IO layer before logging.
        io.updateInputs(inputs);
        Logger.processInputs("DriveBase", inputs);
        // Publish odometry, module states, and chassis speeds for analysis.
        Logger.recordOutput("Odometry/Robot", getPose());
        Logger.recordOutput("SwerveStates/Measured", inputs.moduleStates);
        Logger.recordOutput("SwerveStates/Target", lastRequestedStates);
        Logger.recordOutput("SwerveStates/CurrentStates", inputs.moduleStates);
        Logger.recordOutput("SwerveStates/DesiredStates", lastRequestedStates);
        Logger.recordOutput("SwerveChassisSpeeds/Measured", inputs.chassisSpeeds);
        Logger.recordOutput("SwerveChassisSpeeds/Desired", lastRequestedSpeeds);
        Logger.recordOutput("Swerve/RobotRotation", getPose().getRotation());
        // Update the on-dashboard field widget so drivers can see pose changes.
        fieldDisplay.setRobotPose(getPose());
    }

    /**
     * Returns the current estimated field pose of the robot.
     *
     * @return Current robot pose in meters and radians.
     */
    public Pose2d getPose() {
        if (swerveDrive == null) {
            // Return a safe default when the swerve drive has not been initialized.
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

        // Use the swerve library helper to reset odometry and the gyro together.
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

        // Directly update the pose estimator without rewinding wheel encoders.
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
        // Forward the request to the shared helper that clamps and logs values.
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
        // Wrap the mapping logic so commands can call it lazily every loop.
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
        // Keep the supplier lightweight so it can be polled each scheduler tick.
        return () -> mapDriverOmega(omegaAxisSupplier.get());
    }

    /**
     * Stops all motion and locks the modules in their current orientation.
     */
    public void stop() {
        if (isSubsystemDisabled()) {
            return;
        }

        // Request zero speeds so the drive controller stops issuing motion.
        requestDrive(0.0, 0.0, 0.0);
        if (swerveDrive != null) {
            // Lock the wheels in place to resist being pushed.
            swerveDrive.lockPose();
        }
    }

    /**
     * Provides access to the configured swerve drive for command factories and testing helpers.
     *
     * @return configured {@link SwerveDrive} instance, or {@code null} if initialization failed or subsystem is disabled
     */
    public SwerveDrive getSwerveDrive() {
        // Expose the raw drive for advanced commands or testing helpers.
        return swerveDrive;
    }

    /**
     * Refreshes PID gains and tolerances from tunable config values.
     * <p>
     * Call this when tuning so any SmartDashboard changes take effect without a redeploy.
     * </p>
     */
    private void refreshTunables() {
        if (swerveController != null) {
            // Apply live-tuned heading gains so changes take effect immediately.
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

            // Load the swerve JSONs from the deploy folder so the robot and sim use the same hardware model.
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

            // Build the swerve drive using the maximum speed limit from config.
            swerveDrive                    = new SwerveParser(configDirectory)
                    .createSwerveDrive(config.getMaximumLinearSpeedMetersPerSecond().get());

            if (isSimulation()) {
                // Simulation safety: disable corrections that assume real-world friction and inertia.
                swerveDrive.setHeadingCorrection(false);
                swerveDrive.setCosineCompensator(false);
                // Keep telemetry verbose in sim to help students see more detail.
                SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
            }

            // Cache the controller so we can tune its PID gains at runtime.
            swerveController = swerveDrive.swerveController;
            swerveController.thetaController.setTolerance(config.getRotationToleranceRadians().get(), 0.1);
            swerveController.thetaController.setPID(
                    config.getHeadingKp().get(),
                    config.getHeadingKi().get(),
                    config.getHeadingKd().get());

            // Default to brake mode so the robot resists rolling when no command is active.
            swerveDrive.setMotorIdleMode(true);

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

        // Clamp the requested speeds so driver commands stay within safe limits.
        Translation2d translation = clampTranslation(new Translation2d(vxMetersPerSecond, vyMetersPerSecond));
        double        rotation    = clampRotation(omegaRadiansPerSecond);

        // Convert the field-relative request into robot-relative chassis speeds.
        lastRequestedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                swerveDrive.getOdometryHeading());
        // Compute the desired module states so we can log what the drive wants each wheel to do.
        lastRequestedStates = swerveDrive.toServeModuleStates(lastRequestedSpeeds, true);

        // Send the final request to the swerve library (field-relative, no open-loop).
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
        // Compute the vector length so we can cap diagonal speeds correctly.
        double magnitude = requestedVelocity.getNorm();
        double maxSpeed  = config.getMaximumLinearSpeedMetersPerSecond().get();

        if (magnitude == 0.0 || magnitude <= maxSpeed) {
            // No scaling needed if we are already within the configured limit.
            return requestedVelocity;
        }

        // Scale the vector back to the max speed while preserving direction.
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
        // Clamp the spin rate so we never exceed the configured maximum.
        double maxRotationSpeed = config.getMaximumAngularSpeedRadiansPerSecond().get();
        return MathUtil.clamp(omegaRadiansPerSecond, -maxRotationSpeed, maxRotationSpeed);
    }

}
