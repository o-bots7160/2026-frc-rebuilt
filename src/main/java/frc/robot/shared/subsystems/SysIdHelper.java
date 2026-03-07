package frc.robot.shared.subsystems;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.shared.logging.Logger;

/**
 * Factory for building SysId routines with consistent logging and unit handling.
 * <p>
 * Subsystems can call
 * {@link #createSimpleRoutine(AbstractSubsystem, String, Consumer, Supplier, Runnable, DoubleSupplier, DoubleSupplier, double, double)}
 * to obtain a ready-to-run {@link SysIdRoutine} that drives a single motor and records voltage, position, and velocity
 * to AdvantageKit.
 * </p>
 */
public final class SysIdHelper {

    /**
     * Creates a SysId routine that commands a motor with raw voltage and logs the measured state.
     *
     * @param subsystem                        subsystem that owns the motor; used for command requirements
     * @param motorLabel                       name to attach to the motor in the SysId log (e.g., "Turret/motor")
     * @param applyVoltage                     consumer that forwards a voltage request to the motor controller
     * @param measuredVoltageSupplier          supplier that reports the currently applied voltage in units.Volts
     * @param updateInputs                     runnable that refreshes sensor data before each SysId log sample
     * @param positionRadiansSupplier          supplier of the mechanism position in radians (angular mechanisms)
     * @param velocityRadiansPerSecondSupplier supplier of the mechanism velocity in radians per second (angular mechanisms)
     * @param rampRateVoltsPerSecond           quasistatic voltage ramp rate in volts per second
     * @param stepVoltage                      step voltage applied during dynamic tests in volts
     * @return configured {@link SysIdRoutine} ready to emit dynamic and quasistatic commands
     */
    public static SysIdRoutine createSimpleRoutine(
            AbstractSubsystem<?> subsystem,
            String motorLabel,
            Consumer<Voltage> applyVoltage,
            Supplier<Voltage> measuredVoltageSupplier,
            Runnable updateInputs,
            DoubleSupplier positionRadiansSupplier,
            DoubleSupplier velocityRadiansPerSecondSupplier,
            double rampRateVoltsPerSecond,
            double stepVoltage) {

        return createSimpleRoutine(
                subsystem, motorLabel, applyVoltage, measuredVoltageSupplier,
                updateInputs, positionRadiansSupplier, velocityRadiansPerSecondSupplier,
                () -> 0.0, rampRateVoltsPerSecond, stepVoltage);
    }

    /**
     * Creates a SysId routine that commands a motor with raw voltage, logs the measured state, and records diagnostic
     * telemetry including raw motor RPM.
     *
     * @param subsystem                        subsystem that owns the motor; used for command requirements
     * @param motorLabel                       name to attach to the motor in the SysId log (e.g., "Turret/motor")
     * @param applyVoltage                     consumer that forwards a voltage request to the motor controller
     * @param measuredVoltageSupplier          supplier that reports the currently applied voltage in units.Volts
     * @param updateInputs                     runnable that refreshes sensor data before each SysId log sample
     * @param positionRadiansSupplier          supplier of the mechanism position in radians (angular mechanisms)
     * @param velocityRadiansPerSecondSupplier supplier of the mechanism velocity in radians per second (angular mechanisms)
     * @param rawMotorRpmSupplier              supplier of the raw encoder velocity in motor RPM for diagnostic logging
     * @param rampRateVoltsPerSecond           quasistatic voltage ramp rate in volts per second
     * @param stepVoltage                      step voltage applied during dynamic tests in volts
     * @return configured {@link SysIdRoutine} ready to emit dynamic and quasistatic commands
     */
    public static SysIdRoutine createSimpleRoutine(
            AbstractSubsystem<?> subsystem,
            String motorLabel,
            Consumer<Voltage> applyVoltage,
            Supplier<Voltage> measuredVoltageSupplier,
            Runnable updateInputs,
            DoubleSupplier positionRadiansSupplier,
            DoubleSupplier velocityRadiansPerSecondSupplier,
            DoubleSupplier rawMotorRpmSupplier,
            double rampRateVoltsPerSecond,
            double stepVoltage) {

        SysIdRoutine.Config config = new SysIdRoutine.Config(
                Units.Volts.per(Units.Second).of(rampRateVoltsPerSecond),
                Units.Volts.of(stepVoltage),
                Units.Seconds.of(120));

        Logger sysIdLog = Logger.getInstance(SysIdHelper.class);

        return new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        applyVoltage,
                        log -> {
                            updateInputs.run();
                            double posRad       = positionRadiansSupplier.getAsDouble();
                            double velRadPerSec = velocityRadiansPerSecondSupplier.getAsDouble();
                            Voltage measuredV   = measuredVoltageSupplier.get();

                            log.motor(motorLabel)
                                    .voltage(measuredV)
                                    .angularPosition(Units.Radians.of(posRad))
                                    .angularVelocity(Units.RadiansPerSecond.of(velRadPerSec));

                            // Diagnostic telemetry so AdvantageScope shows the same values SysId logs.
                            sysIdLog.recordOutput("sysIdVelocityRadPerSec", velRadPerSec);
                            sysIdLog.recordOutput("sysIdVelocityRpm",
                                    edu.wpi.first.math.util.Units.radiansPerSecondToRotationsPerMinute(velRadPerSec));
                            sysIdLog.recordOutput("sysIdRawMotorRpm", rawMotorRpmSupplier.getAsDouble());
                            sysIdLog.recordOutput("sysIdPositionRad", posRad);
                            sysIdLog.recordOutput("sysIdVoltage", measuredV.in(Units.Volts));
                        },
                        subsystem));
    }

    private SysIdHelper() {
    }
}
