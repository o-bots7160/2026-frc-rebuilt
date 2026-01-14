package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import swervelib.SwerveDrive;

/**
 * YAGSL-backed implementation that pulls pose, module states, and gyro data from the configured swerve drive.
 */
public class DriveBaseIOYagsl implements DriveBaseIO {

    private final SwerveDrive swerveDrive;

    /**
     * Creates an IO bridge that mirrors telemetry from the provided YAGSL swerve drive into AdvantageKit.
     *
     * @param swerveDrive active YAGSL swerve drive instance supplying sensor data
     */
    public DriveBaseIOYagsl(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void updateInputs(DriveBaseIOInputs inputs) {
        inputs.pose                     = swerveDrive.getPose();
        inputs.chassisSpeeds            = swerveDrive.getFieldVelocity();
        inputs.moduleStates             = cloneStates(swerveDrive.getStates());
        inputs.moduleTargets            = new SwerveModuleState[0];
        inputs.gyroYaw                  = swerveDrive.getYaw();
        inputs.odometryTimestampSeconds = Timer.getFPGATimestamp();
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
