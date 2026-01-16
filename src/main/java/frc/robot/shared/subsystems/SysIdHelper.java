package frc.robot.shared.subsystems;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Factory for building SysId routines with consistent logging and unit handling.
 * <p>
 * Subsystems can call {@link #createSimpleRoutine(AbstractSubsystem, String, Consumer, Supplier, DoubleSupplier, DoubleSupplier)} to obtain a
 * ready-to-run {@link SysIdRoutine} that drives a single motor and records voltage, position, and velocity to AdvantageKit.
 * </p>
 */
public final class SysIdHelper {

    /**
     * Creates a SysId routine that commands a motor with raw voltage and logs the measured state.
     *
     * @param subsystem               subsystem that owns the motor; used for command requirements
     * @param motorLabel              name to attach to the motor in the SysId log (e.g., "Turret/motor")
     * @param applyVoltage            consumer that forwards a voltage request to the motor controller
     * @param measuredVoltageSupplier supplier that reports the currently applied voltage in units.Volts
     * @param positionSupplier        supplier of the mechanism position in radians (angular mechanisms)
     * @param velocitySupplier        supplier of the mechanism velocity in radians per second (angular mechanisms)
     * @return configured {@link SysIdRoutine} ready to emit dynamic and quasistatic commands
     */
    public static SysIdRoutine createSimpleRoutine(
            AbstractSubsystem<?> subsystem,
            String motorLabel,
            Consumer<Voltage> applyVoltage,
            Supplier<Voltage> measuredVoltageSupplier,
            DoubleSupplier positionSupplier,
            DoubleSupplier velocitySupplier) {

        return new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        applyVoltage,
                        log -> log.motor(motorLabel)
                                .voltage(measuredVoltageSupplier.get())
                                .angularPosition(Units.Radians.of(positionSupplier.getAsDouble()))
                                .angularVelocity(Units.RadiansPerSecond.of(velocitySupplier.getAsDouble())),
                        subsystem));
    }

    private SysIdHelper() {
    }
}
