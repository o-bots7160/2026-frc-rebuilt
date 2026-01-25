package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.devices.Motor;
import frc.robot.shared.subsystems.AbstractSetAndSeekSubsystem;
import frc.robot.subsystems.turret.config.TurretSubsystemConfig;
import frc.robot.subsystems.turret.devices.TurretMotor;
import frc.robot.subsystems.turret.devices.TurretSimMotor;

/**
 * Turret subsystem with a single profiled motor. Exposes the set-and-seek API so callers can drive to angles in degrees while the superclass handles
 * motion profiling, limits, and logging.
 */
public class TurretSubsystem extends AbstractSetAndSeekSubsystem<TurretSubsystemConfig> {
    private static Motor buildMotor(TurretSubsystemConfig config) {
        if (!config.enabled) {
            return null;
        }

        return RobotBase.isReal()
                ? TurretMotor.create(config.turretMotorConfig)
                : TurretSimMotor.create(
                        config.turretMotorConfig,
                        config.getMaximumVelocitySupplier(),
                        config.getMaximumAccelerationSupplier());
    }

    /**
     * Builds the turret subsystem with a single SparkMax-driven motor and default motion profile values.
     *
     * @param config turret configuration bundle loaded from JSON; angles are expressed in degrees
     */
    public TurretSubsystem(TurretSubsystemConfig config) {
        this(config, buildMotor(config));
    }

    private TurretSubsystem(TurretSubsystemConfig config, Motor motor) {
        super(config, motor);
    }

    /**
     * Polls for tunable updates and refreshes the hardware motor configuration when needed.
     * <p>
     * This keeps SparkMax settings aligned with Elastic/NetworkTables edits without requiring a restart.
     * </p>
     */
    @Override
    public void periodic() {
        if (!isFMSAttached() && motor instanceof TurretMotor turretMotor) {
            turretMotor.refreshConfiguration();
        }
    }

}
