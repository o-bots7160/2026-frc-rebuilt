package frc.robot.shared.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.shared.config.AbstractSetAndSeekSubsystemConfig;

/**
 * Base subsystem that generates and follows a trapezoidal motion profile.
 * <p>
 * Concrete mechanisms should extend this class to gain a simple "set and seek" API: call {@link #setTarget(double)} to define a goal and repeatedly
 * call {@link #seekTarget()} from a command to step the profile forward. Motor control is intentionally left as a no-op so hardware bindings can be added
 * later.
 * </p>
 */
public abstract class AbstractSetAndSeekSubsystem<TConfig extends AbstractSetAndSeekSubsystemConfig> extends AbstractSubsystem<TConfig> {
    protected TrapezoidProfile.Constraints constraints;

    protected TrapezoidProfile             profile;

    protected TrapezoidProfile.State       goalState;

    protected TrapezoidProfile.State       setpointState;

    /**
     * Creates a profiled subsystem with bounded setpoints and motion constraints.
     *
     * @param config Configuration values that define the allowable range, motion limits, and initial state.
     */
    protected AbstractSetAndSeekSubsystem(TConfig config) {
        super(config);

        constraints   = new TrapezoidProfile.Constraints(config.getMaximumVelocitySupplier().get(),
                config.getMaximumAccelerationSupplier().get());
        profile       = new TrapezoidProfile(constraints);

        double initialPosition = config.getInitialPositionSupplier().get();
        double initialVelocity = config.getInitialVelocitySupplier().get();

        goalState     = new TrapezoidProfile.State(initialPosition, 0.0);
        setpointState = new TrapezoidProfile.State(initialPosition, initialVelocity);
    }

    /**
     * Sets a new goal for the motion profile. Values outside the configured range are clamped.
     *
     * @param targetPosition Desired mechanism position (same units as the configuration).
     */
    public void setTarget(double targetPosition) {
        if (isSubsystemDisabled()) {
            return;
        }

        double clampedTarget = clamp(targetPosition, config.getMinimumSetpointSupplier().get(),
                config.getMaximumSetpointSupplier().get());
        goalState = new TrapezoidProfile.State(clampedTarget, 0.0);
    }

    /**
    * Advances the trapezoidal profile by one cycle and hands the setpoint to the subclass for actuation.
     */
    public void seekTarget() {
        if (isSubsystemDisabled()) {
            return;
        }

        refreshConstraints();

        setpointState = profile.calculate(kDt, setpointState, goalState);
        applySetpoint(setpointState);
        logSetpoint(setpointState, goalState);
    }

    /**
     * Resets the internal profile state to a measured pose. Call this before starting a new profile to avoid jumps when encoders drift.
     *
     * @param position Current measured position in mechanism units.
     * @param velocity Current measured velocity in mechanism units per second.
     */
    public void synchronizeToMeasurement(double position, double velocity) {
        setpointState = new TrapezoidProfile.State(position, velocity);
        goalState     = new TrapezoidProfile.State(position, 0.0);
    }

    /**
     * States whether the mechanism is within the configured tolerance of the goal position.
     *
     * @return True when the measured position is at the goal.
     */
    public boolean atTarget() {
        if (isSubsystemDisabled()) {
            return true;
        }

        double error = Math.abs(goalState.position - getMeasuredPosition());
        return error <= config.getPositionToleranceSupplier().get();
    }

    /**
     * Hook for subclasses to respond when a seek command is interrupted. Default implementation does nothing.
     */
    public void handleSeekInterrupted() {
        // TODO: add mechanism-specific stop or brake behavior when hardware is wired.
    }

    /**
     * Applies the calculated setpoint to hardware. Override to route the desired position/velocity to motor controllers.
     *
     * @param setpoint The next state from the trapezoidal profile.
     */
    protected void applySetpoint(TrapezoidProfile.State setpoint) {
        // TODO: implement motor control once hardware bindings are available.
    }

    /**
     * Provides the measured mechanism position. Override to read from an encoder or other sensor.
     *
     * @return The current measured position in mechanism units. Defaults to the profiled setpoint for simulation-only usage.
     */
    protected double getMeasuredPosition() {
        return setpointState.position;
    }

    /**
     * Provides the measured mechanism velocity. Override to read from an encoder or other sensor.
     *
     * @return The current measured velocity in mechanism units per second. Defaults to the profiled setpoint velocity.
     */
    protected double getMeasuredVelocity() {
        return setpointState.velocity;
    }

    private void logSetpoint(TrapezoidProfile.State setpoint, TrapezoidProfile.State goal) {
        log.recordOutput("goalPosition", goal.position);
        log.recordOutput("goalVelocity", goal.velocity);
        log.recordOutput("setpointPosition", setpoint.position);
        log.recordOutput("setpointVelocity", setpoint.velocity);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void refreshConstraints() {
        constraints = new TrapezoidProfile.Constraints(config.getMaximumVelocitySupplier().get(),
                config.getMaximumAccelerationSupplier().get());
        profile     = new TrapezoidProfile(constraints);
    }
}