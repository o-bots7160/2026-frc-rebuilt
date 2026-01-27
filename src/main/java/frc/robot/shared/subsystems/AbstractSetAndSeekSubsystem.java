package frc.robot.shared.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.devices.motor.DisabledMotor;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorIOInputsAutoLogged;
import frc.robot.shared.config.AbstractSetAndSeekSubsystemConfig;

/**
 * Base subsystem that generates and follows a trapezoidal motion profile.
 * <p>
 * Concrete mechanisms should extend this class to gain a simple "set and seek" API: call {@link #setTarget(double)} to define a goal and repeatedly
 * call {@link #seekTarget()} from a command to step the profile forward. Motor control is intentionally left as a no-op so hardware bindings can be
 * added later.
 * </p>
 */
public abstract class AbstractSetAndSeekSubsystem<TConfig extends AbstractSetAndSeekSubsystemConfig> extends AbstractSubsystem<TConfig> {
    protected final Motor                   motor;

    protected final MotorIOInputsAutoLogged motorInputs = new MotorIOInputsAutoLogged();

    protected TrapezoidProfile.Constraints  constraints;

    protected final ProfiledPIDController   controller;

    protected final SimpleMotorFeedforward  feedforward;

    protected TrapezoidProfile.State        goalState;

    protected TrapezoidProfile.State        setpointState;

    private final SysIdRoutine              sysIdRoutine;

    /**
     * Creates a profiled subsystem with bounded setpoints, motion constraints, and a single motor.
     *
     * @param config Configuration values that define the allowable range, motion limits, and initial state.
     * @param motor  Motor controller that reports position/velocity and accepts duty-cycle commands, or null to use a disabled no-op motor.
     */
    protected AbstractSetAndSeekSubsystem(TConfig config, Motor motor) {
        super(config);
        // Use a no-op motor when hardware is absent so the subsystem can still run safely in sim or disabled mode.
        this.motor  = motor != null ? motor : new DisabledMotor();

        // Trapezoid profile constraints define the max cruise speed and acceleration for smooth motion.
        constraints = new TrapezoidProfile.Constraints(
                config.getMaximumVelocityDegreesPerSecondSupplier().get(),
                config.getMaximumAccelerationDegreesPerSecondSquaredSupplier().get());

        // Profiled PID drives the mechanism toward the goal while respecting the trapezoid limits.
        controller  = new ProfiledPIDController(
                config.getkPSupplier().get(),
                config.getkISupplier().get(),
                config.getkDSupplier().get(),
                constraints,
                kDt);
        // Position tolerance is in mechanism units; velocity tolerance is a small fraction of max speed.
        controller.setTolerance(config.getPositionToleranceDegreesSupplier().get(),
                config.getMaximumVelocityDegreesPerSecondSupplier().get() * 0.05);

        // Feedforward estimates the voltage needed to maintain a desired velocity/acceleration.
        feedforward = new SimpleMotorFeedforward(
                config.getkSSupplier().get(),
                config.getkVSupplier().get(),
                config.getkASupplier().get());

        // Seed the profile with the configured starting position/velocity so the first update is stable.
        double initialPosition = config.getInitialPositionDegreesSupplier().get();
        double initialVelocity = config.getInitialVelocityDegreesPerSecondSupplier().get();

        // The goal state is where we want to end; the setpoint state is the next step along the profile.
        goalState     = new TrapezoidProfile.State(initialPosition, 0.0);
        setpointState = new TrapezoidProfile.State(initialPosition, initialVelocity);

        // Reset the controller to the current state and then give it the initial goal.
        controller.reset(initialPosition, initialVelocity);
        controller.setGoal(goalState);

        // SysId routine is used by characterization commands to identify feedforward gains.
        sysIdRoutine = SysIdHelper.createSimpleRoutine(
                this,
                className + "/motor",
                this.motor::setVoltage,
                this.motor::getVoltage,
            () -> this.motor.updateInputs(motorInputs),
            () -> motorInputs.positionRads,
            () -> motorInputs.velocityRadPerSec);
    }

    /**
     * Sets a new goal for the motion profile. Values outside the configured range are clamped.
     *
     * @param targetPosition Desired mechanism position (same units as the configuration).
     */
    public void setTarget(double targetPosition) {
        if (isSubsystemDisabled()) {
            logDisabled("setTarget");
            return;
        }

        // Capture inputs for easier debugging.
        double minimumSetpointDegrees = config.getMinimumSetpointDegreesSupplier().get();
        double maximumSetpointDegrees = config.getMaximumSetpointDegreesSupplier().get();

        // Clamp the request so we never ask the mechanism to move past its safe range.
        double clampedTarget = clamp(targetPosition, minimumSetpointDegrees, maximumSetpointDegrees);
        log.recordOutput("targetRequestedPosition", targetPosition);
        log.recordOutput("targetClampedPosition", clampedTarget);
        log.recordOutput("targetWasClamped", targetPosition != clampedTarget);
        // Store the goal with zero velocity so the profile knows where to stop.
        goalState = new TrapezoidProfile.State(clampedTarget, 0.0);

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
        motor.updateInputs(motorInputs);
        org.littletonrobotics.junction.Logger.processInputs(className + "/motor", motorInputs);

        // Pull the latest motion constraints so live tuning takes effect immediately.
        refreshConstraints();

        // Use the profiled PID to calculate the next output from the current position.
        double measuredPosition = getMeasuredPosition();
        double controllerOutput = controller.calculate(measuredPosition);

        // Grab the setpoint the profile wants us to follow this cycle.
        setpointState = controller.getSetpoint();

        // Feedforward estimates the volts needed to maintain the desired velocity.
        double feedforwardVolts = feedforward.calculate(setpointState.velocity);
        double voltageCommand   = controllerOutput + feedforwardVolts;
        double positionError    = goalState.position - measuredPosition;

        log.recordOutput("positionErrorDegrees", positionError);
        log.recordOutput("controllerOutputVolts", controllerOutput);
        log.recordOutput("feedforwardVolts", feedforwardVolts);
        log.recordOutput("voltageCommandVolts", voltageCommand);

        applySetpoint(setpointState, voltageCommand);
        logSetpoint(setpointState, goalState);
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

        log.recordOutput("settleMeasuredPosition", measuredPosition);
        log.recordOutput("settleMeasuredVelocity", measuredVelocity);

        // Reset clears the internal error so the profile starts from the real motion state.
        controller.reset(measuredPosition, measuredVelocity);
        // Reuse setTarget to clamp and update the goal.
        setTarget(measuredPosition);
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
     * Hook for subclasses to respond when a seek command is interrupted. Default implementation stops the motor.
     */
    public void handleSeekInterrupted() {
        motor.stop();
    }

    /**
     * Applies the calculated setpoint to hardware. Override to customize control behavior.
     *
     * @param setpoint       the next state from the trapezoidal profile
     * @param voltageCommand requested motor voltage in volts before clamping
     */
    protected void applySetpoint(TrapezoidProfile.State setpoint, double voltageCommand) {
        double clampedVoltage = clamp(voltageCommand, -12.0, 12.0);
        motor.setVoltage(clampedVoltage);

        log.recordOutput("commandedVoltage", clampedVoltage);
    }

    /**
     * Provides the measured mechanism position. Override to read from an encoder or other sensor.
     *
     * @return The current measured position in mechanism units. Defaults to the profiled setpoint for simulation-only usage.
     */
    protected double getMeasuredPosition() {
        return motor.getEncoderPosition();
    }

    /**
     * Provides the measured mechanism velocity. Override to read from an encoder or other sensor.
     *
     * @return The current measured velocity in mechanism units per second. Defaults to the profiled setpoint velocity.
     */
    protected double getMeasuredVelocity() {
        return motor.getEncoderVelocity();
    }

    private void logSetpoint(TrapezoidProfile.State setpoint, TrapezoidProfile.State goal) {
        // Log the profile targets and the live sensor feedback each cycle.
        log.recordOutput("goalPosition", goal.position);
        log.recordOutput("goalVelocity", goal.velocity);
        log.recordOutput("setpointPosition", setpoint.position);
        log.recordOutput("setpointVelocity", setpoint.velocity);
        log.recordOutput("measuredPosition", getMeasuredPosition());
        log.recordOutput("measuredVelocity", getMeasuredVelocity());
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void refreshConstraints() {
        // Read live tunable limits so the profile respects current max speed and acceleration.
        constraints = new TrapezoidProfile.Constraints(
                config.getMaximumVelocityDegreesPerSecondSupplier().get(),
                config.getMaximumAccelerationDegreesPerSecondSquaredSupplier().get());
        controller.setConstraints(constraints);
    }
}