package frc.robot.shared.subsystems;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.devices.motor.Motor;
import frc.robot.shared.config.AbstractVelocitySubsystemConfig;
import frc.robot.shared.config.MotorConfig;
import frc.robot.shared.config.RobotEnvironment;
import frc.robot.shared.config.VelocityMotionConfig;

/**
 * Base subsystem for mechanisms that maintain a target velocity using feedforward and PID control.
 * <p>
 * All RPM values in the public API represent mechanism (flywheel) speed after gear reduction, not motor shaft speed. The gear ratio is applied at the
 * motor encoder conversion layer, so subsystem code never deals with motor-side rotations. Internally, all velocity math uses radians per second to
 * align with WPILib conventions.
 * </p>
 * <p>
 * When {@code maximumAccelerationRpmPerSecond} in config is greater than zero, the subsystem uses a trapezoidal profile to ramp velocity smoothly.
 * When it is zero, the subsystem applies direct PID + feedforward control without a ramp.
 * </p>
 *
 * @param <TConfig> configuration type supplying velocity limits, PID gains, and feedforward gains
 */
public abstract class AbstractVelocitySubsystem<TConfig extends AbstractVelocitySubsystemConfig>
        extends AbstractMotorSubsystem<TConfig> {

    /**
     * Functional interface for a three-argument function used by {@link #buildVelocityMotor}.
     *
     * @param <A> first argument type
     * @param <B> second argument type
     * @param <C> third argument type
     * @param <R> return type
     */
    @FunctionalInterface
    protected interface TriFunction<A, B, C, R> {

        /**
         * Applies this function to the given arguments.
         *
         * @param a first argument
         * @param b second argument
         * @param c third argument
         * @return function result
         */
        R apply(A a, B b, C c);
    }

    /**
     * Builds the correct motor implementation for the current environment using the shared velocity subsystem pattern.
     * <p>
     * Returns null when the config is disabled so the parent falls back to {@link frc.robot.devices.motor.DisabledMotor}. Real and simulation motors
     * are selected using the provided factory functions.
     * </p>
     *
     * @param <TMotorConfig> concrete motor config type
     * @param config         velocity subsystem config supplying max velocity and acceleration for sim motor suppliers
     * @param motorConfig    motor-specific configuration bundle (CAN ID, gear ratio, inversion, etc.)
     * @param realFactory    factory function that creates the real motor from a motor config
     * @param simFactory     factory function that creates the sim motor from a motor config and velocity/acceleration suppliers
     * @return configured motor, or null when the subsystem is disabled
     */
    protected static <TMotorConfig extends MotorConfig> Motor buildVelocityMotor(
            AbstractVelocitySubsystemConfig config,
            TMotorConfig motorConfig,
            Function<TMotorConfig, Motor> realFactory,
            TriFunction<TMotorConfig, Supplier<Double>, Supplier<Double>, Motor> simFactory) {
        if (!config.enabled) {
            return null;
        }

        return RobotEnvironment.isReal()
                ? realFactory.apply(motorConfig)
                : simFactory.apply(
                        motorConfig,
                        () -> VelocityMotionConfig.rpmToDegreesPerSecond(config.motionProfile.getMaximumVelocityRpm()),
                        () -> VelocityMotionConfig.rpmToDegreesPerSecond(config.motionProfile.getMaximumAccelerationRpmPerSecond()));
    }

    /**
     * PID controller that corrects velocity error.
     */
    protected final PIDController  controller;

    /**
     * Target velocity in radians per second. Zero means stopped.
     */
    private double                 targetVelocityRadPerSec;

    /**
     * Current velocity setpoint being tracked after the optional trapezoidal ramp.
     */
    private double                 setpointVelocityRadPerSec;

    /**
     * Previous cycle's velocity setpoint in radians per second, used by {@code calculateWithVelocities} so the feedforward accounts for acceleration
     * (kA).
     */
    private double                 previousSetpointVelocityRadPerSec;

    /**
     * Optional trapezoidal profile for smooth velocity ramps. Null when acceleration limit is zero (direct control).
     */
    private TrapezoidProfile       velocityProfile;

    /**
     * Current profile state used when the trapezoidal ramp is active.
     */
    private TrapezoidProfile.State profileState;

    /**
     * Timer that tracks how long the measured velocity has been within tolerance of the target.
     */
    private final Timer            settledTimer = new Timer();

    /**
     * True when the measured velocity is currently within tolerance of the target.
     */
    private boolean                withinTolerance;

    /**
     * Creates a velocity-controlled subsystem with PID, feedforward, and optional trapezoidal velocity ramp.
     *
     * @param config configuration bundle supplying velocity limits, PID gains, and feedforward gains
     * @param motor  motor controller that reports velocity and accepts voltage commands, or null to use a disabled no-op motor
     */
    protected AbstractVelocitySubsystem(TConfig config, Motor motor) {
        super(config, motor);

        controller                        = new PIDController(
                config.pid.getkP(),
                config.pid.getkI(),
                config.pid.getkD(),
                kDt);
        applyAntiWindup(config);

        targetVelocityRadPerSec           = 0.0;
        setpointVelocityRadPerSec         = 0.0;
        previousSetpointVelocityRadPerSec = 0.0;
        withinTolerance                   = false;

        rebuildVelocityProfile();
        settledTimer.start();
    }

    /**
     * Refreshes motor, feedforward, and PID configuration when not attached to the FMS, and logs common velocity telemetry.
     * <p>
     * Override this if you need additional periodic behavior, but call {@code super.periodic()} to keep live tuning updates and common telemetry
     * active.
     * </p>
     */
    @Override
    public void periodic() {
        super.periodic();
        if (!isFMSAttached()) {
            refreshVelocityController();
        }

        log.recordOutput("isReady", isReady());
        log.recordOutput("measuredRpm", getMeasuredVelocityRpm());
        log.recordOutput("targetRpm", getTargetVelocityRpm());
    }

    /**
     * Sets a new target velocity for the mechanism. Values are clamped to the configured maximum in both directions.
     * <p>
     * The target is in mechanism RPM (after gear reduction). Positive values spin the mechanism forward; negative values spin it in reverse.
     * Requested and clamped values are logged for tuning visibility.
     * </p>
     * <p>
     * The settle timer and tolerance flag are only reset when the measured velocity is outside tolerance of the new target. This prevents
     * reset-flicker in distance-based commands where the interpolated target drifts slightly each cycle while the mechanism is already tracking
     * within tolerance. Large target jumps (e.g., idle to full speed) still trigger a full reset and settle window.
     * </p>
     *
     * @param targetRpm desired mechanism velocity in RPM (0 to stop, positive for forward, negative for reverse)
     */
    public void setTargetVelocityRpm(double targetRpm) {
        if (isSubsystemDisabled()) {
            logDisabled("setTargetVelocityRpm");
            return;
        }

        double  maximumRpm = config.motionProfile.getMaximumVelocityRpm();
        double  clampedRpm = MathUtil.clamp(targetRpm, -maximumRpm, maximumRpm);
        boolean wasClamped = targetRpm != clampedRpm;

        log.recordVerboseOutput("targetRequestedRpm", targetRpm);
        log.recordVerboseOutput("targetClampedRpm", clampedRpm);
        log.recordVerboseOutput("targetWasClamped", wasClamped);

        double newTargetRadPerSec    = Units.rotationsPerMinuteToRadiansPerSecond(clampedRpm);

        // Only reset the profile and settle tracking when the target actually changes.
        // Repeated calls with the same target (e.g., from createContinuousVelocityCommand)
        // must not restart the ramp or the profile will never fully converge.
        // Use a small epsilon because tunable reads may introduce floating-point noise.
        double targetChangeRadPerSec = Math.abs(newTargetRadPerSec - targetVelocityRadPerSec);
        if (targetChangeRadPerSec < 1e-6) {
            return;
        }

        targetVelocityRadPerSec = newTargetRadPerSec;

        // Reset profile state so the ramp starts from the actual measured velocity.
        if (velocityProfile != null) {
            profileState = new TrapezoidProfile.State(getMeasuredVelocityRadiansPerSecond(), 0.0);
        }

        // Only reset settle tracking when the measured velocity is outside tolerance
        // of the new target. Distance-based commands update the target slightly each
        // cycle as the robot moves; unconditionally resetting here causes the
        // withinTolerance field to flicker false even though the flywheel is keeping
        // up with the drifting setpoint. Preserving tolerance state for small target
        // changes eliminates the flicker while still enforcing a full settle window
        // after large jumps (e.g., idle to full speed).
        double  measuredRadPerSec      = getMeasuredVelocityRadiansPerSecond();
        double  errorToNewTargetRadSec = Math.abs(newTargetRadPerSec - measuredRadPerSec);
        double  toleranceRadPerSec     = config.motionProfile.getVelocityToleranceRadiansPerSecond();
        boolean stillWithinTolerance   = errorToNewTargetRadSec <= toleranceRadPerSec;

        if (!stillWithinTolerance) {
            withinTolerance = false;
            settledTimer.reset();
            settledTimer.start();
        }

        log.recordVerboseOutput("tolerancePreservedOnTargetChange", stillWithinTolerance);
    }

    /**
     * Advances the velocity controller by one cycle: updates motor inputs, steps the optional velocity ramp, computes PID + feedforward, and applies
     * the voltage to the motor.
     */
    public void seekVelocity() {
        if (isSubsystemDisabled()) {
            logDisabled("seekVelocity");
            return;
        }

        double measuredVelocityRadPerSec = getMeasuredVelocityRadiansPerSecond();

        // Step the velocity profile if a trapezoidal ramp is active.
        if (velocityProfile != null) {
            TrapezoidProfile.State goalState = new TrapezoidProfile.State(targetVelocityRadPerSec, 0.0);
            profileState              = velocityProfile.calculate(kDt, profileState, goalState);
            setpointVelocityRadPerSec = profileState.position;
        } else {
            setpointVelocityRadPerSec = targetVelocityRadPerSec;
        }

        // PID corrects for velocity error.
        double pidOutput        = controller.calculate(measuredVelocityRadPerSec, setpointVelocityRadPerSec);

        // Feedforward estimates the voltage needed to transition from the previous setpoint
        // to the current setpoint. Using calculateWithVelocities (discrete plant inversion)
        // ensures the kA term accounts for acceleration during velocity ramps.
        double feedforwardVolts = feedforward.calculateWithVelocities(
                previousSetpointVelocityRadPerSec, setpointVelocityRadPerSec);
        previousSetpointVelocityRadPerSec = setpointVelocityRadPerSec;
        double voltageCommand         = pidOutput + feedforwardVolts;

        double velocityErrorRadPerSec = targetVelocityRadPerSec - measuredVelocityRadPerSec;
        double velocityErrorRpm       = Units.radiansPerSecondToRotationsPerMinute(velocityErrorRadPerSec);

        log.recordVerboseOutput("velocityErrorRpm", velocityErrorRpm);
        log.recordVerboseOutput("pidOutputVolts", pidOutput);
        log.recordVerboseOutput("feedforwardVolts", feedforwardVolts);
        log.recordOutput("voltageCommandVolts", voltageCommand);
        log.recordVerboseOutput("setpointRpm", Units.radiansPerSecondToRotationsPerMinute(setpointVelocityRadPerSec));

        // Track settle status.
        double  toleranceRadPerSec = config.motionProfile.getVelocityToleranceRadiansPerSecond();
        boolean currentlyWithin    = Math.abs(velocityErrorRadPerSec) <= toleranceRadPerSec;
        if (!currentlyWithin) {
            withinTolerance = false;
            settledTimer.reset();
            settledTimer.start();
        } else {
            withinTolerance = true;
        }

        applyVoltage(voltageCommand);
    }

    /**
     * Reports whether the measured velocity is within tolerance of the target and has been stable for the configured settle time.
     *
     * @return true when the mechanism is at the target velocity and has been stable long enough
     */
    public boolean isAtTargetVelocity() {
        boolean atTarget = withinTolerance && settledTimer.hasElapsed(config.motionProfile.getSettleTimeSeconds());
        log.recordVerboseOutput("atTargetVelocity", atTarget);
        return atTarget;
    }

    /**
     * Reports whether the measured velocity is currently within tolerance of the target, without requiring the settle timer.
     * <p>
     * Use this for continuously-updating targets (such as distance-based RPM) where the settle timer resets each time the target changes.
     * {@link #isAtTargetVelocity()} is still preferred for fixed-target scenarios where stability over time matters.
     * </p>
     *
     * @return true when the measured velocity is within tolerance of the current target right now
     */
    public boolean isWithinTolerance() {
        log.recordVerboseOutput("withinTolerance", withinTolerance);
        return withinTolerance;
    }

    /**
     * Reports whether the subsystem is ready for downstream consumers.
     * <p>
     * This is a stable, uniform API that all velocity subsystems expose. Composite commands and cross-subsystem suppliers should prefer this method
     * over subsystem-specific readiness checks so wiring code stays consistent across mechanisms.
     * </p>
     *
     * @return true when the mechanism is at the target velocity and has been stable long enough
     */
    public boolean isReady() {
        return isAtTargetVelocity();
    }

    /**
     * Returns the current target velocity in RPM.
     *
     * @return target velocity in mechanism RPM
     */
    public double getTargetVelocityRpm() {
        return Units.radiansPerSecondToRotationsPerMinute(targetVelocityRadPerSec);
    }

    /**
     * Returns the current measured velocity in RPM.
     *
     * @return measured velocity in mechanism RPM
     */
    public double getMeasuredVelocityRpm() {
        return Units.radiansPerSecondToRotationsPerMinute(getMeasuredVelocityRadiansPerSecond());
    }

    /**
     * Returns the configured idle velocity in RPM.
     * <p>
     * Commands can use this to set the target to the idle speed without direct config access.
     * </p>
     *
     * @return idle velocity in mechanism RPM from config
     */
    public double getIdleVelocityRpm() {
        return config.motionProfile.getIdleVelocityRpm();
    }

    /**
     * Resets velocity controller state to zero and stops the motor.
     * <p>
     * Clears the target velocity, setpoint, tolerance flag, and trapezoidal profile state so the mechanism is fully idle.
     * </p>
     */
    @Override
    public void stop() {
        // Temporary diagnostic: log who called stop() so we can trace unexpected shutdowns.
        log.warning("stop() called — stack: " + getCallerInfo());

        targetVelocityRadPerSec           = 0.0;
        setpointVelocityRadPerSec         = 0.0;
        previousSetpointVelocityRadPerSec = 0.0;
        withinTolerance                   = false;
        if (velocityProfile != null) {
            profileState = new TrapezoidProfile.State(0.0, 0.0);
        }
        super.stop();
    }

    /**
     * Returns a short caller trace for diagnostic logging.
     *
     * @return caller class and method from the stack trace
     */
    private String getCallerInfo() {
        StackTraceElement[] stack = Thread.currentThread().getStackTrace();
        StringBuilder       sb    = new StringBuilder();
        // Skip getStackTrace, getCallerInfo, stop — show the next 5 frames.
        for (int i = 3; i < Math.min(stack.length, 8); i++) {
            if (sb.length() > 0) {
                sb.append(" <- ");
            }
            sb.append(stack[i].getClassName().substring(stack[i].getClassName().lastIndexOf('.') + 1))
                    .append(".")
                    .append(stack[i].getMethodName())
                    .append(":")
                    .append(stack[i].getLineNumber());
        }
        return sb.toString();
    }

    /**
     * Rebuilds the optional trapezoidal velocity profile from the current config.
     * <p>
     * The profile is used to ramp velocity, so profile "position" represents motor velocity in radians per second. The profile's {@code maxVelocity}
     * constraint controls the rate of velocity change (motor acceleration), and jerk limiting is disabled for a simple linear ramp.
     * </p>
     */
    private void rebuildVelocityProfile() {
        double maxAccelRadPerSecSq = config.motionProfile.getMaximumAccelerationRadiansPerSecondSquared();
        if (maxAccelRadPerSecSq > 0.0) {
            velocityProfile = new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(maxAccelRadPerSecSq, Double.POSITIVE_INFINITY));
            profileState    = new TrapezoidProfile.State(setpointVelocityRadPerSec, 0.0);
        } else {
            velocityProfile = null;
            profileState    = null;
        }
    }

    /**
     * Re-reads PID gains and velocity constraints from config so live tuning updates take effect immediately.
     */
    private void refreshVelocityController() {
        controller.setPID(
                config.pid.getkP(),
                config.pid.getkI(),
                config.pid.getkD());
        applyAntiWindup(config);
        rebuildVelocityProfile();
    }

    /**
     * Applies IZone and integrator range limits from config to the PID controller.
     * <p>
     * When {@code iZone} is greater than zero, the integral accumulator resets whenever the error exceeds the threshold, preventing windup during
     * large transients. When {@code integratorRangeMax} is greater than zero, the integral output is clamped to the specified voltage range.
     * </p>
     *
     * @param config configuration bundle supplying anti-windup parameters
     */
    private void applyAntiWindup(TConfig config) {
        double iZone = config.pid.getIZone();
        if (iZone > 0.0) {
            controller.setIZone(iZone);
        } else {
            controller.setIZone(Double.POSITIVE_INFINITY);
        }

        double integratorRangeMax = config.pid.getIntegratorRangeMax();
        if (integratorRangeMax > 0.0) {
            controller.setIntegratorRange(-integratorRangeMax, integratorRangeMax);
        }
    }
}
