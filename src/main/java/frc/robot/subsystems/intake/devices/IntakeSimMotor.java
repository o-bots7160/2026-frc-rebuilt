package frc.robot.subsystems.intake.devices;

import java.util.function.Supplier;

import frc.robot.devices.motor.AbstractVelocitySimMotor;
import frc.robot.subsystems.intake.config.IntakeMotorConfig;

/**
 * Simulation-only intake motor wrapper configured from the intake subsystem config.
 * <p>
 * Use this class when running in simulation so the roller velocity behavior can be exercised without hardware. Construct it with the same config used
 * by the subsystem to ensure gearing matches the real device.
 * </p>
 */
public class IntakeSimMotor extends AbstractVelocitySimMotor<IntakeMotorConfig> {

    /**
     * Creates a simulated intake motor wrapper using the supplied configuration values.
     * <p>
     * Use this factory so the call site stays consistent with the hardware motor creation pattern.
     * </p>
     *
     * @param motorConfig                 intake motor configuration bundle loaded from JSON
     * @param maximumVelocitySupplier     supplier yielding the maximum mechanism velocity in degrees per second
     * @param maximumAccelerationSupplier supplier yielding the maximum mechanism acceleration in degrees per second squared
     * @return configured simulated intake motor wrapper
     */
    public static IntakeSimMotor create(
            IntakeMotorConfig motorConfig,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        return new IntakeSimMotor(motorConfig, maximumVelocitySupplier, maximumAccelerationSupplier);
    }

    /**
     * Creates a simulated intake motor using the supplied configuration values.
     *
     * @param motorConfig                 intake motor configuration bundle loaded from JSON
     * @param maximumVelocitySupplier     supplier yielding the maximum mechanism velocity in degrees per second
     * @param maximumAccelerationSupplier supplier yielding the maximum mechanism acceleration in degrees per second squared
     */
    private IntakeSimMotor(
            IntakeMotorConfig motorConfig,
            Supplier<Double> maximumVelocitySupplier,
            Supplier<Double> maximumAccelerationSupplier) {
        super(
                "IntakeMotor",
                motorConfig,
                maximumVelocitySupplier,
                maximumAccelerationSupplier);
    }
}
