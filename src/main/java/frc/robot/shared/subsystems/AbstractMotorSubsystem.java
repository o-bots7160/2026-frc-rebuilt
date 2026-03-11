package frc.robot.shared.subsystems;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.devices.motor.DisabledMotor;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorIOInputsAutoLogged;
import frc.robot.shared.config.AbstractMotorSubsystemConfig;

/**
 * Base subsystem for any mechanism driven by a single motor with feedforward estimation and SysId support.
 * <p>
 * Position-based (set-and-seek) and velocity-based subsystems both extend this class to share motor lifecycle management, voltage clamping,
 * feedforward configuration, SysId routine creation, and AdvantageKit motor input logging. Concrete subclasses add their own controller (profiled PID
 * for position, standard PID for velocity) and expose an appropriate public API.
 * </p>
 *
 * @param <TConfig> configuration type that supplies PID and feedforward gains
 */
public abstract class AbstractMotorSubsystem<TConfig extends AbstractMotorSubsystemConfig> extends AbstractSubsystem<TConfig> {

    /**
     * Motor used to drive the mechanism.
     */
    protected final Motor                   motor;

    /**
     * Logged motor inputs snapshot used for telemetry and replay.
     */
    protected final MotorIOInputsAutoLogged motorInputs = new MotorIOInputsAutoLogged();

    /**
     * Feedforward model used to estimate the voltage needed to maintain a desired velocity.
     */
    protected SimpleMotorFeedforward        feedforward;

    /**
     * SysId routine for characterization commands.
     */
    private final SysIdRoutine              sysIdRoutine;

    /**
     * Creates a motor-driven subsystem with feedforward estimation and SysId support.
     *
     * @param config configuration bundle supplying PID and feedforward gains
     * @param motor  motor controller that reports position/velocity and accepts voltage commands, or null to use a disabled no-op motor
     */
    protected AbstractMotorSubsystem(TConfig config, Motor motor) {
        super(config);

        // Use a no-op motor when hardware is absent so the subsystem can still run safely in sim or disabled mode.
        this.motor   = motor != null ? motor : new DisabledMotor();

        // Feedforward estimates the voltage needed to maintain a desired velocity/acceleration.
        feedforward  = new SimpleMotorFeedforward(
                config.feedforward.getkS(),
                config.feedforward.getkV(),
                config.feedforward.getkA());

        // SysId routine is used by characterization commands to identify feedforward gains.
        sysIdRoutine = SysIdHelper.createSimpleRoutine(
                this,
                className + "/motor",
                this.motor::setVoltage,
                this.motor::getVoltage,
                () -> this.motor.updateInputs(motorInputs),
                () -> motorInputs.positionRadians,
                () -> motorInputs.velocityRadPerSec,
                () -> motorInputs.velocityMotorRpm,
                config.sysId.getRampRateVoltsPerSecond(),
                config.sysId.getStepVoltage());
    }

    /**
     * Refreshes feedforward configuration when not attached to the FMS, and logs motor inputs every cycle.
     * <p>
     * Motor inputs (encoder position, velocity, current, temperature, etc.) are captured each cycle so telemetry is always available, even when no
     * command is actively driving the mechanism. Override this if you need additional periodic behavior, but call {@code super.periodic()} to keep
     * live tuning updates and motor input logging active.
     * </p>
     */
    @Override
    public void periodic() {
        if (!isFMSAttached()) {
            refreshFeedforward();
        }

        updateAndLogMotorInputs();
    }

    /**
     * Exposes the underlying SysId routine so command factories can build characterization commands without subsystems manufacturing commands.
     *
     * @return configured SysId routine for the primary motor
     */
    public SysIdRoutine getSysIdRoutine() {
        return sysIdRoutine;
    }

    /**
     * Logs the start of a SysId routine for operator awareness.
     * <p>
     * Call this once before running a SysId command sequence so the console reflects the upcoming characterization sweep.
     * </p>
     */
    public void logSysIdStart() {
        log.info("Start SysID Routine");
    }

    /**
     * Logs the successful completion of a SysId routine.
     * <p>
     * Call this after a SysId command finishes to confirm the characterization sweep completed.
     * </p>
     */
    public void logSysIdEnd() {
        log.info("End SysID Routine");
    }

    /**
     * Logs when a SysId routine is interrupted before completion.
     * <p>
     * Call this when a SysId command is cancelled or interrupted so operators know the sweep did not finish.
     * </p>
     */
    public void logSysIdInterrupted() {
        log.warning("Interrupted SysID Routine");
    }

    /**
     * Returns the current measured position of the mechanism in radians.
     *
     * @return measured position in radians
     */
    public double getMeasuredPositionRadians() {
        return motor.getPositionRadians();
    }

    /**
     * Returns the current measured velocity of the mechanism in radians per second.
     *
     * @return measured velocity in radians per second
     */
    public double getMeasuredVelocityRadiansPerSecond() {
        return motor.getVelocityRadiansPerSecond();
    }

    /**
     * Stops the motor and resets subsystem state to a safe idle condition.
     * <p>
     * Subclasses override this method to zero out controller state (velocity targets, profile goals, etc.) and then call {@code super.stop()} so the
     * motor is always halted at the end of the chain.
     * </p>
     */
    public void stop() {
        motor.stop();
    }

    /**
     * Refreshes motor sensor data and logs it via AdvantageKit for telemetry and replay.
     * <p>
     * Call this from the concrete subsystem's control loop before computing PID/feedforward so the controller reads fresh sensor values.
     * </p>
     */
    protected void updateAndLogMotorInputs() {
        motor.updateInputs(motorInputs);
        log.processInputs("motor", motorInputs);
    }

    /**
     * Clamps a voltage command to the safe range (±12V) and sends it to the motor.
     *
     * @param voltageCommand requested motor voltage in volts before clamping
     */
    protected void applyVoltage(double voltageCommand) {
        Voltage clampedVoltage = Volts.of(MathUtil.clamp(voltageCommand, -12.0, 12.0));
        motor.setVoltage(clampedVoltage);
    }

    /**
     * Re-reads feedforward gains from config so live tuning updates affect voltage estimates immediately.
     */
    protected void refreshFeedforward() {
        feedforward = new SimpleMotorFeedforward(
                config.feedforward.getkS(),
                config.feedforward.getkV(),
                config.feedforward.getkA());
    }
}
