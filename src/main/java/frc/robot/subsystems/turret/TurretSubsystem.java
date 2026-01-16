package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.devices.Motor;
import frc.robot.devices.motor.SimMotor;
import frc.robot.shared.subsystems.AbstractSetAndSeekSubsystem;
import frc.robot.subsystems.turret.config.TurretSubsystemConfig;
import frc.robot.subsystems.turret.devices.TurretMotor;

public class TurretSubsystem extends AbstractSetAndSeekSubsystem<TurretSubsystemConfig> {
    private static Motor buildMotor(TurretSubsystemConfig config) {
        if (RobotBase.isReal()) {
            return new TurretMotor(config);
        }

    double motorRotationsPerMechanismRotation = config.getMotorRotationsPerMechanismRotationSupplier().get();

    return new SimMotor(
        "TurretMotorSim",
        motorRotationsPerMechanismRotation,
        config.getMinimumSetpointSupplier(),
        config.getMaximumSetpointSupplier(),
        config.getMaximumVelocitySupplier(),
        config.getMaximumAccelerationSupplier());
    }

    /**
     * Builds the turret subsystem with a single SparkMax-driven motor and default motion profile values.
     *
     * @param config turret configuration bundle loaded from JSON
     */
    public TurretSubsystem(TurretSubsystemConfig config) {
        this(config, buildMotor(config));
    }

    private TurretSubsystem(TurretSubsystemConfig config, Motor motor) {
        super(config, motor);
    }

}
