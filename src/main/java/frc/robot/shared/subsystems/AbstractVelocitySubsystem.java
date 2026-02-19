package frc.robot.shared.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.devices.motor.Motor;
import frc.robot.shared.config.AbstractVelocitySubsystemConfig;

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

        controller                = new PIDController(
                config.getkP(),
                config.getkI(),
                config.getkD(),
                kDt);

        targetVelocityRadPerSec   = 0.0;
        setpointVelocityRadPerSec = 0.0;
        withinTolerance           = false;

        rebuildVelocityProfile();
        settledTimer.start();
    }

    /**
     * Refreshes motor, feedforward, and PID configuration when not attached to the FMS.
     * <p>
     * Override this if you need additional periodic behavior, but call {@code super.periodic()} to keep live tuning updates active.
     * </p>
     */
    @Override
    public void periodic() {
        super.periodic();
        if (!isFMSAttached()) {
            refreshVelocityController();
        }
    }

    /**
     * Sets a new target velocity for the mechanism. Values are clamped to the configured maximum in both directions.
     * <p>
     * The target is in mechanism RPM (after gear reduction). Positive values spin the mechanism forward; negative values spin it in reverse.
     * Requested and clamped values are logged for tuning visibility.
     * </p>
     *
     * @param targetRpm desired mechanism velocity in RPM (0 to stop, positive for forward, negative for reverse)
     */
    public void setTargetVelocityRpm(double targetRpm) {
        if (isSubsystemDisabled()) {
            logDisabled("setTargetVelocityRpm");
            return;
        }

        double  maximumRpm = config.getMaximumVelocityRpm();
        double  clampedRpm = MathUtil.clamp(targetRpm, -maximumRpm, maximumRpm);
        boolean wasClamped = targetRpm != clampedRpm;

        log.recordVerboseOutput("targetRequestedRpm", targetRpm);
        log.recordVerboseOutput("targetClampedRpm", clampedRpm);
        log.recordVerboseOutput("targetWasClamped", wasClamped);

        targetVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(clampedRpm);

        // Reset profile state so the ramp starts from the actual measured velocity.
        if (velocityProfile != null) {
            profileState = new TrapezoidProfile.State(getMeasuredVelocityRadiansPerSecond(), 0.0);
        }

        // Reset settle tracking since we have a new target.
        withinTolerance = false;
        settledTimer.reset();
        settledTimer.start();
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

        // Refresh sensor data and log it.
        updateAndLogMotorInputs();

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
        double pidOutput              = controller.calculate(measuredVelocityRadPerSec, setpointVelocityRadPerSec);

        // Feedforward estimates the voltage needed to maintain the setpoint velocity.
        double feedforwardVolts       = feedforward.calculate(setpointVelocityRadPerSec);
        double voltageCommand         = pidOutput + feedforwardVolts;

        double velocityErrorRadPerSec = targetVelocityRadPerSec - measuredVelocityRadPerSec;
        double velocityErrorRpm       = Units.radiansPerSecondToRotationsPerMinute(velocityErrorRadPerSec);

        log.recordOutput("velocityErrorRpm", velocityErrorRpm);
        log.recordVerboseOutput("pidOutputVolts", pidOutput);
        log.recordVerboseOutput("feedforwardVolts", feedforwardVolts);
        log.recordOutput("voltageCommandVolts", voltageCommand);
        log.recordOutput("targetRpm", getTargetVelocityRpm());
        log.recordOutput("measuredRpm", getMeasuredVelocityRpm());
        log.recordOutput("setpointRpm", Units.radiansPerSecondToRotationsPerMinute(setpointVelocityRadPerSec));

        // Track settle status.
        double  toleranceRadPerSec = config.getVelocityToleranceRadiansPerSecond();
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
        boolean atTarget = withinTolerance && settledTimer.hasElapsed(config.getSettleTimeSeconds());
        log.recordOutput("atTargetVelocity", atTarget);
        return atTarget;
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
        return config.getIdleVelocityRpm();
    }

    /**
     * Stops the motor and resets the target velocity to zero.
     */
    public void stopMechanism() {
        targetVelocityRadPerSec   = 0.0;
        setpointVelocityRadPerSec = 0.0;
        withinTolerance           = false;
        if (velocityProfile != null) {
            profileState = new TrapezoidProfile.State(0.0, 0.0);
        }
        stopMotor();
    }

    /**
     * Rebuilds the optional trapezoidal velocity profile from the current config.
     * <p>
     * The profile is used to ramp velocity, so profile "position" represents motor velocity in radians per second. The profile's
     * {@code maxVelocity} constraint controls the rate of velocity change (motor acceleration), and jerk limiting is disabled for a simple linear
     * ramp.
     * </p>
     */
    private void rebuildVelocityProfile() {
        double maxAccelRadPerSecSq = config.getMaximumAccelerationRadiansPerSecondSquared();
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
                config.getkP(),
                config.getkI(),
                config.getkD());
        rebuildVelocityProfile();
    }
}
