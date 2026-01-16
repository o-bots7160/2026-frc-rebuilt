package frc.robot.shared.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.devices.Motor;
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
     * @param motor  Motor controller that reports position/velocity and accepts duty-cycle commands.
     */
    protected AbstractSetAndSeekSubsystem(TConfig config, Motor motor) {
        super(config);
        this.motor  = motor;

        constraints = new TrapezoidProfile.Constraints(config.getMaximumVelocitySupplier().get(),
                config.getMaximumAccelerationSupplier().get());

        controller  = new ProfiledPIDController(
                config.getkPSupplier().get(),
                config.getkISupplier().get(),
                config.getkDSupplier().get(),
                constraints,
                kDt);
        controller.setTolerance(config.getPositionToleranceSupplier().get(),
                config.getMaximumVelocitySupplier().get() * 0.05);

        feedforward = new SimpleMotorFeedforward(
                config.getkSSupplier().get(),
                config.getkVSupplier().get(),
                config.getkASupplier().get());

        double initialPosition = config.getInitialPositionSupplier().get();
        double initialVelocity = config.getInitialVelocitySupplier().get();

        goalState     = new TrapezoidProfile.State(initialPosition, 0.0);
        setpointState = new TrapezoidProfile.State(initialPosition, initialVelocity);

        controller.reset(initialPosition, initialVelocity);
        controller.setGoal(goalState);

        sysIdRoutine = SysIdHelper.createSimpleRoutine(
                this,
                className + "/motor",
                motor::setVoltage,
                motor::getVoltage,
                this::getMeasuredPosition,
                this::getMeasuredVelocity);
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

        controller.setGoal(goalState);
    }

    /**
     * Advances the trapezoidal profile by one cycle and hands the setpoint to the subclass for actuation.
     */
    public void seekTarget() {
        if (isSubsystemDisabled()) {
            return;
        }

        motor.updateInputs(motorInputs);
        org.littletonrobotics.junction.Logger.processInputs(className + "/motor", motorInputs);

        refreshConstraints();

        double measuredPosition = getMeasuredPosition();
        double controllerOutput = controller.calculate(measuredPosition);

        setpointState = controller.getSetpoint();

        double feedforwardVolts = feedforward.calculate(setpointState.velocity);
        double voltageCommand   = controllerOutput + feedforwardVolts;

        applySetpoint(setpointState, voltageCommand);
        logSetpoint(setpointState, goalState);
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
     * Exposes the underlying SysId routine so command factories can build characterization commands without subsystems manufacturing commands.
     *
     * @return configured SysId routine for the primary motor
     */
    public SysIdRoutine getSysIdRoutine() {
        return sysIdRoutine;
    }

    /**
     * Retargets the motion profile to the current measured position so the profiled controller decelerates to a stop.
     */
    public void holdCurrentPosition() {
        setTarget(getMeasuredPosition());
    }

    /**
     * States whether the profiled controller is within both position and velocity tolerances of its goal.
     *
     * @return True when the profile reports settled.
     */
    public boolean isProfileSettled() {
        return controller.atGoal();
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
        constraints = new TrapezoidProfile.Constraints(config.getMaximumVelocitySupplier().get(),
                config.getMaximumAccelerationSupplier().get());
        controller.setConstraints(constraints);
    }
}