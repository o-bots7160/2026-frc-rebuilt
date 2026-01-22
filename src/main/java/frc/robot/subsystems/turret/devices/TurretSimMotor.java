package frc.robot.subsystems.turret.devices;

import frc.robot.devices.motor.SimMotor;
import frc.robot.subsystems.turret.config.TurretSubsystemConfig;

/**
 * Simulation-only turret motor wrapper configured from the turret subsystem config.
 * <p>
 * Use this class when running in simulation so the turret profile behavior can be exercised without hardware. Construct it with the same config
 * used by the subsystem to ensure limits and gearing match the real device.
 * </p>
 */
public class TurretSimMotor extends SimMotor {

    /**
     * Creates a simulated turret motor using the supplied configuration values.
     *
     * @param config turret configuration bundle loaded from JSON; provides gearing, limits, and motion constraints in degrees
     */
    public TurretSimMotor(TurretSubsystemConfig config) {
        super(
                "TurretMotorSim",
                config.getMotorRotationsPerMechanismRotationSupplier().get(),
                config.getMinimumSetpointSupplier(),
                config.getMaximumSetpointSupplier(),
                config.getMaximumVelocitySupplier(),
                config.getMaximumAccelerationSupplier());
    }
}
