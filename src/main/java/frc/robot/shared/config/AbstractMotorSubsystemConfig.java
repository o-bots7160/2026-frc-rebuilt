package frc.robot.shared.config;

/**
 * Configuration values shared by any motor-driven subsystem that uses PID control and feedforward estimation.
 * <p>
 * Concrete configs for position-based (set-and-seek) and velocity-based subsystems extend this class to inherit tunable PID and feedforward gains.
 * Gains are organized into nested bundles: {@link PidConfig}, {@link FeedforwardConfig}, and {@link SysIdRoutineConfig}. Each bundle is surfaced to
 * SmartDashboard under the parent subsystem's namespace (e.g., {@code ShooterSubsystem/pid/kP}).
 * </p>
 */
public abstract class AbstractMotorSubsystemConfig extends AbstractSubsystemConfig {

    /** PID controller gains for this motor subsystem. */
    public PidConfig         pid         = new PidConfig();

    /** Feedforward gains for this motor subsystem. */
    public FeedforwardConfig feedforward = new FeedforwardConfig();

    /** SysId routine parameters for this motor subsystem. */
    public SysIdRoutineConfig sysId      = new SysIdRoutineConfig();
}
