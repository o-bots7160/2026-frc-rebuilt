package frc.robot.shared.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.devices.motor.Motor;
import frc.robot.shared.config.AbstractSetAndSeekSubsystemConfig;

/**
 * Base subsystem that generates and follows a trapezoidal motion profile.
 * <p>
 * Concrete mechanisms should extend this class to gain a simple "set and seek" API: call {@link #setTarget(double)} to define a goal and repeatedly
 * call {@link #seekTarget()} from a command to step the profile forward. Motor control, feedforward, SysId support, and AdvantageKit input logging
 * are provided by the {@link AbstractMotorSubsystem} parent class.
 * </p>
 */
public abstract class AbstractSetAndSeekSubsystem<TConfig extends AbstractSetAndSeekSubsystemConfig>
        extends AbstractMotorSubsystem<TConfig> {

    /**
     * Trapezoid constraints that bound velocity and acceleration.
     */
    protected TrapezoidProfile.Constraints constraints;

    /**
     * Profiled PID controller that tracks the trapezoid setpoint.
     */
    protected final ProfiledPIDController  controller;

    /**
     * Desired goal state for the trapezoid profile.
     */
    protected TrapezoidProfile.State       goalState;

    /**
     * Current setpoint state produced by the trapezoid profile.
     */
    protected TrapezoidProfile.State       setpointState;

    /**
     * Creates a profiled subsystem with bounded setpoints, motion constraints, and a single motor.
     *
     * @param config Configuration values that define the allowable range, motion limits, and initial state.
     * @param motor  Motor controller that reports position/velocity and accepts duty-cycle commands, or null to use a disabled no-op motor.
     */
    protected AbstractSetAndSeekSubsystem(TConfig config, Motor motor) {
        super(config, motor);

        // Trapezoid profile constraints define the max cruise speed and acceleration for smooth motion.
        constraints = new TrapezoidProfile.Constraints(
                config.motionProfile.getMaximumVelocityRadiansPerSecond(),
                config.motionProfile.getMaximumAccelerationRadiansPerSecondSquared());

        // Profiled PID drives the mechanism toward the goal while respecting the trapezoid limits.
        controller  = new ProfiledPIDController(
                config.pid.getkP(),
                config.pid.getkI(),
                config.pid.getkD(),
                constraints,
                kDt);

        // Position tolerance is in mechanism units; velocity tolerance is a small fraction of max speed.
        controller.setTolerance(
                config.motionProfile.getPositionToleranceRadians(),
                config.motionProfile.getVelocityToleranceRadiansPerSecond());

        // Seed the profile with the configured starting position/velocity so the first update is stable.
        double initialPosition = config.motionProfile.getInitialPositionRadians();
        double initialVelocity = config.motionProfile.getInitialVelocityRadiansPerSecond();

        // The goal state is where we want to end; the setpoint state is the next step along the profile.
        goalState     = new TrapezoidProfile.State(initialPosition, 0.0);
        setpointState = new TrapezoidProfile.State(initialPosition, initialVelocity);

        // Reset the controller to the current state and then give it the initial goal.
        controller.reset(initialPosition, initialVelocity);
        controller.setGoal(goalState);

        // Sync the motor encoder with the configured starting position so the
        // measured position matches the profile seed on power-up.
        this.motor.setEncoderPosition(initialPosition);
    }

    /**
     * Refreshes motor configuration and profile constraints when not attached to the FMS.
     * <p>
     * Override this if you need additional periodic behavior, but call {@code super.periodic()} to keep live tuning updates active.
     * </p>
     */
    @Override
    public void periodic() {
        super.periodic();
        if (!isFMSAttached()) {
            refreshConstraints();
        }
    }

    /**
     * Sets a new goal for the motion profile. Values outside the configured range are clamped.
     *
     * @param targetPositionDegrees desired mechanism position in degrees
     */
    public void setTarget(double targetPositionDegrees) {
        if (isSubsystemDisabled()) {
            logDisabled("setTarget");
            return;
        }

        // Capture inputs for easier debugging.
        double minimumSetpointRadians = config.getMinimumSetpointRadians();
        double maximumSetpointRadians = config.getMaximumSetpointRadians();
        double requestedTargetRadians = Units.degreesToRadians(targetPositionDegrees);

        // Clamp the request so we never ask the mechanism to move past its safe range.
        double clampedTargetRadians   = MathUtil.clamp(requestedTargetRadians, minimumSetpointRadians, maximumSetpointRadians);
        log.recordVerboseOutput("targetRequestedPositionDegrees", targetPositionDegrees);
        log.recordVerboseOutput("targetClampedPositionDegrees", Units.radiansToDegrees(clampedTargetRadians));
        log.recordVerboseOutput("targetWasClamped", requestedTargetRadians != clampedTargetRadians);
        // Store the goal with zero velocity so the profile knows where to stop.
        goalState = new TrapezoidProfile.State(clampedTargetRadians, 0.0);

        // Give the goal to the controller so the next seek step advances toward it.
        controller.setGoal(goalState);
    }

    /**
     * Advances the trapezoidal profile by one cycle and hands the setpoint to the subclass for actuation.
     */
    public void seekTarget() {
        if (isSubsystemDisabled()) {
            logDisabled("seekTarget");
            return;
        }

        // Refresh sensor data and log it before we compute the next setpoint.
        updateAndLogMotorInputs();

        // Use the profiled PID to calculate the next output from the current position.
        double measuredPosition         = getMeasuredPosition();

        // Capture the current setpoint velocity before advancing the profile so the
        // discrete feedforward can compute the acceleration (kA) term.
        double previousSetpointVelocity = controller.getSetpoint().velocity;
        double controllerOutput         = controller.calculate(measuredPosition);

        // Grab the setpoint the profile wants us to follow this cycle.
        setpointState = controller.getSetpoint();

        // Feedforward estimates the volts needed to reach the next setpoint velocity.
        // Subclasses can override calculateFeedforward to use a different model (e.g., ArmFeedforward).
        double feedforwardVolts = calculateFeedforward(previousSetpointVelocity, setpointState);
        double voltageCommand   = controllerOutput + feedforwardVolts;
        double positionError    = goalState.position - measuredPosition;

        log.recordVerboseOutput("positionErrorDegrees", Units.radiansToDegrees(positionError));
        log.recordVerboseOutput("controllerOutputVolts", controllerOutput);
        log.recordVerboseOutput("feedforwardVolts", feedforwardVolts);
        log.recordOutput("voltageCommandVolts", voltageCommand);

        applySetpoint(setpointState, voltageCommand);

        log.recordVerboseOutput("goalPositionDegrees", Units.radiansToDegrees(goalState.position));
        log.recordVerboseOutput("goalVelocityDegreesPerSec", Units.radiansToDegrees(goalState.velocity));
        log.recordVerboseOutput("setpointPositionDegrees", Units.radiansToDegrees(setpointState.position));
        log.recordVerboseOutput("setpointVelocityDegreesPerSec", Units.radiansToDegrees(setpointState.velocity));
    }

    /**
     * Resets the profiled controller and goal state, then stops the motor.
     * <p>
     * Use this for a full stop that clears all profile state. For a light-touch interruption that preserves profile state (so settle commands can
     * decelerate smoothly), use {@link #handleSeekInterrupted()} instead.
     * </p>
     */
    @Override
    public void stop() {
        double measuredPosition = getMeasuredPosition();
        double measuredVelocity = getMeasuredVelocity();
        goalState     = new TrapezoidProfile.State(measuredPosition, 0.0);
        setpointState = new TrapezoidProfile.State(measuredPosition, measuredVelocity);
        controller.reset(measuredPosition, measuredVelocity);
        controller.setGoal(goalState);
        super.stop();
    }

    /**
     * Resets the motor encoder to the configured initial position and reinitializes the profile state.
     * <p>
     * Call this from a dashboard button or utility command after manually repositioning a mechanism back to its starting position (e.g., stowing a
     * harvester arm or centering a turret by hand). The motor is stopped before the reset to avoid applying stale voltage at the new position.
     * </p>
     */
    public void resetEncoderPosition() {
        if (isSubsystemDisabled()) {
            logDisabled("resetEncoderPosition");
            return;
        }

        double initialPosition = config.motionProfile.getInitialPositionRadians();

        // Stop motion before resetting so no stale voltage is applied.
        motor.stop();
        motor.setEncoderPosition(initialPosition);

        // Reinitialize all profile state so the controller starts fresh from the initial position.
        goalState     = new TrapezoidProfile.State(initialPosition, 0.0);
        setpointState = new TrapezoidProfile.State(initialPosition, 0.0);
        controller.reset(initialPosition, 0.0);
        controller.setGoal(goalState);

        log.recordOutput("encoderReset", true);
    }

    /**
     * Hook for subclasses to respond when a seek command is interrupted. Default implementation stops the motor without resetting profile state.
     * <p>
     * Settle commands depend on the preserved profile state to decelerate smoothly. If you need a full reset, call {@link #stop()} instead.
     * </p>
     */
    public void handleSeekInterrupted() {
        motor.stop();
    }

    /**
     * Retargets the profile using the latest measured position and velocity to settle without oscillation.
     * <p>
     * Call this when interrupting a moving command so the controller decelerates smoothly from the current motion state.
     * </p>
     */
    public void settleAtCurrentPosition() {
        if (isSubsystemDisabled()) {
            logDisabled("settleAtCurrentPosition");
            return;
        }

        // Capture the live state so the controller can decelerate smoothly.
        double measuredPosition = getMeasuredPosition();
        double measuredVelocity = getMeasuredVelocity();

        log.recordVerboseOutput("settleMeasuredPositionDegrees", Units.radiansToDegrees(measuredPosition));
        log.recordVerboseOutput("settleMeasuredVelocityDegreesPerSec", Units.radiansToDegrees(measuredVelocity));

        // Reset clears the internal error so the profile starts from the real motion state.
        controller.reset(measuredPosition, measuredVelocity);
        // Reuse setTarget to clamp and update the goal (convert to degrees for API consistency).
        setTarget(Units.radiansToDegrees(measuredPosition));
    }

    /**
     * States whether the profiled controller is within both position and velocity tolerances of its goal.
     *
     * @return True when the profile reports settled.
     */
    public boolean isProfileSettled() {
        // The controller checks both position and velocity tolerances for a true stop.
        boolean settled = controller.atGoal();
        log.recordOutput("profileSettled", settled);
        return settled;
    }

    /**
     * Returns the current measured position in degrees.
     * <p>
     * Delegates to {@link #getMeasuredPosition()} and converts from radians. Use this for external consumers such as suppliers and telemetry.
     * </p>
     *
     * @return measured mechanism position in degrees
     */
    public double getMeasuredPositionDegrees() {
        return Units.radiansToDegrees(getMeasuredPosition());
    }

    /**
     * Computes the feedforward voltage for the current profile step.
     * <p>
     * The default implementation uses the parent's {@link edu.wpi.first.math.controller.SimpleMotorFeedforward} with discrete-time plant inversion
     * (the two-velocity form, so kA contributes the acceleration torque the profile demands). Subclasses that need a different feedforward model
     * (e.g., {@link edu.wpi.first.math.controller.ArmFeedforward} for gravity compensation) should override this method.
     * </p>
     *
     * @param previousSetpointVelocity velocity setpoint from the previous cycle in radians per second
     * @param currentSetpoint          the setpoint state the profile wants us to follow this cycle
     * @return feedforward voltage in volts
     */
    protected double calculateFeedforward(double previousSetpointVelocity, TrapezoidProfile.State currentSetpoint) {
        return feedforward.calculateWithVelocities(previousSetpointVelocity, currentSetpoint.velocity);
    }

    /**
     * Applies the calculated setpoint to hardware. Override to customize control behavior.
     *
     * @param setpoint       the next state from the trapezoidal profile
     * @param voltageCommand requested motor voltage in volts before clamping
     */
    protected void applySetpoint(TrapezoidProfile.State setpoint, double voltageCommand) {
        applyVoltage(voltageCommand);
    }

    /**
     * Provides the measured mechanism position. Override to read from an encoder or other sensor.
     *
     * @return The current measured position in radians. Defaults to the motor's reported position.
     */
    protected double getMeasuredPosition() {
        return getMeasuredPositionRadians();
    }

    /**
     * Provides the measured mechanism velocity. Override to read from an encoder or other sensor.
     *
     * @return The current measured velocity in radians per second. Defaults to the motor's reported velocity.
     */
    protected double getMeasuredVelocity() {
        return getMeasuredVelocityRadiansPerSecond();
    }

    private void refreshConstraints() {
        // Read live tunable limits so the profile respects current max speed and acceleration.
        constraints = new TrapezoidProfile.Constraints(
                config.motionProfile.getMaximumVelocityRadiansPerSecond(),
                config.motionProfile.getMaximumAccelerationRadiansPerSecondSquared());
        controller.setConstraints(constraints);

        // Refresh gains so live tuning updates affect the controller immediately.
        controller.setPID(
                config.pid.getkP(),
                config.pid.getkI(),
                config.pid.getkD());

        controller.setTolerance(
                config.motionProfile.getPositionToleranceRadians(),
                config.motionProfile.getVelocityToleranceRadiansPerSecond());
    }
}