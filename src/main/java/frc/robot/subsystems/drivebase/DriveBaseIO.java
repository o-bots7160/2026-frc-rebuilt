package frc.robot.subsystems.drivebase;

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
        public Pose2d              pose                     = new Pose2d();

        public ChassisSpeeds       chassisSpeeds            = new ChassisSpeeds();

        public SwerveModuleState[] moduleStates             = new SwerveModuleState[0];

        public SwerveModuleState[] moduleTargets            = new SwerveModuleState[0];

        public Rotation2d          gyroYaw                  = new Rotation2d();

        public double              odometryTimestampSeconds = 0.0;
    }

    /**
     * Refreshes the inputs structure with the latest state from the drive base sensors.
     *
     * @param inputs mutable inputs container to populate for logging
     */
    void updateInputs(DriveBaseIOInputs inputs);
}
