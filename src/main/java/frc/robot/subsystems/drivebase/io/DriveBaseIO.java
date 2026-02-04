package frc.robot.subsystems.drivebase.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Defines the contract for drive base sensor inputs that AdvantageKit will log. Implementations should pull data from the active drive system
 * (hardware or simulation) and populate the provided inputs structure once per loop.
 */
public interface DriveBaseIO {

    /**
     * Container of all drive base telemetry fields that AdvantageKit will persist automatically.
     */
    @AutoLog
    public static class DriveBaseIOInputs {
        /**
         * Current robot pose estimate in meters and radians.
         */
        public Pose2d              pose                     = new Pose2d();

        /**
         * Measured chassis speeds in meters per second and radians per second.
         */
        public ChassisSpeeds       chassisSpeeds            = new ChassisSpeeds();

        /**
         * Measured swerve module states.
         */
        public SwerveModuleState[] moduleStates             = new SwerveModuleState[0];

        /**
         * Target swerve module states requested by the controller.
         */
        public SwerveModuleState[] moduleTargets            = new SwerveModuleState[0];

        /**
         * Current gyro yaw reading.
         */
        public Rotation2d          gyroYaw                  = new Rotation2d();

        /**
         * Timestamp for the latest odometry update in seconds.
         */
        public double              odometryTimestampSeconds = 0.0;
    }

    /**
     * Refreshes the inputs structure with the latest state from the drive base sensors.
     *
     * @param inputs mutable inputs container to populate for logging
     */
    void updateInputs(DriveBaseIOInputs inputs);
}
