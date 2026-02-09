package frc.robot.subsystems.drivebase.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import swervelib.SwerveDrive;

/**
 * YAGSL-backed implementation that pulls pose, module states, and gyro data from the configured swerve drive.
 */
public class DriveBaseIOYagsl implements DriveBaseIO {

    private final SwerveDrive          swerveDrive;

    /**
     * Standalone odometry tracker that uses only wheel encoders and gyro data.
     * <p>
     * This runs in parallel with YAGSL's internal pose estimator so we can see how far pure
     * odometry drifts from the vision-corrected estimate.
     * </p>
     */
    private final SwerveDriveOdometry rawOdometry;

    /**
     * Creates an IO bridge that mirrors telemetry from the provided YAGSL swerve drive into AdvantageKit.
     *
     * @param swerveDrive active YAGSL swerve drive instance supplying sensor data
     */
    public DriveBaseIOYagsl(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        // Create a separate odometry tracker seeded with the current gyro and module positions.
        this.rawOdometry = new SwerveDriveOdometry(
                swerveDrive.kinematics,
                swerveDrive.getYaw(),
                swerveDrive.getModulePositions(),
                swerveDrive.getPose());
    }

    @Override
    public void updateInputs(DriveBaseIOInputs inputs) {
        inputs.pose                     = swerveDrive.getPose();
        inputs.chassisSpeeds            = swerveDrive.getFieldVelocity();
        inputs.moduleStates             = cloneStates(swerveDrive.getStates());
        inputs.moduleTargets            = new SwerveModuleState[0];
        inputs.gyroYaw                  = swerveDrive.getYaw();
        inputs.odometryTimestampSeconds = Timer.getFPGATimestamp();

        // Update the standalone odometry tracker with the latest gyro and module data.
        rawOdometry.update(swerveDrive.getYaw(), swerveDrive.getModulePositions());
        inputs.odometryOnlyPose = rawOdometry.getPoseMeters();
    }

    /**
     * Resets the raw odometry tracker to a given pose.
     * <p>
     * Call this whenever the main odometry is reset so the two trackers stay in sync at the start.
     * </p>
     *
     * @param pose pose to reset the raw odometry to in meters and radians
     */
    public void resetRawOdometry(Pose2d pose) {
        rawOdometry.resetPosition(
                swerveDrive.getYaw(),
                swerveDrive.getModulePositions(),
                pose);
    }

    private SwerveModuleState[] cloneStates(SwerveModuleState[] states) {
        if (states == null) {
            return new SwerveModuleState[0];
        }
        SwerveModuleState[] copy = new SwerveModuleState[states.length];
        for (int i = 0; i < states.length; i++) {
            copy[i] = states[i];
        }
        return copy;
    }
}
