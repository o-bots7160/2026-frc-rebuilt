package frc.robot.shared.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.shared.subsystems.AbstractVelocitySubsystem;

/**
 * Command factory for velocity-controlled subsystems that exposes common SysId characterization commands while keeping subsystems free of command
 * creation.
 *
 * @param <TSubsystem> concrete velocity subsystem type
 */
public abstract class AbstractVelocityCommandFactory<TSubsystem extends AbstractVelocitySubsystem<?>>
        extends AbstractSubsystemCommandFactory<TSubsystem> {

    private static final double DEFAULT_SYSID_TIMEOUT_SECONDS = 3.0;

    /**
     * Builds a command factory bound to a velocity subsystem.
     *
     * @param subsystem velocity subsystem instance shared by generated commands
     */
    protected AbstractVelocityCommandFactory(TSubsystem subsystem) {
        super(subsystem);
    }

    /**
     * Creates a quasistatic SysId characterization command that ramps voltage slowly in the requested direction.
     *
     * @param direction sweep direction (forward or reverse)
     * @return command that runs the quasistatic test until completion
     */
    public Command createSysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return wrapSysIdCommand(subsystem.getSysIdRoutine().quasistatic(direction));
    }

    /**
     * Creates a quasistatic SysId command with a timeout to prevent runaway motion.
     *
     * @param direction      sweep direction (forward or reverse)
     * @param timeoutSeconds maximum duration before the command ends
     * @return timed quasistatic SysId command
     */
    public Command createSysIdQuasistaticCommand(SysIdRoutine.Direction direction, double timeoutSeconds) {
        return createSysIdQuasistaticCommand(direction).withTimeout(timeoutSeconds);
    }

    /**
     * Creates a quasistatic SysId command with a default timeout (3 seconds) to safeguard robot motion.
     *
     * @param direction sweep direction (forward or reverse)
     * @return timed quasistatic SysId command
     */
    public Command createSysIdQuasistaticCommandWithDefaultTimeout(SysIdRoutine.Direction direction) {
        return createSysIdQuasistaticCommand(direction, DEFAULT_SYSID_TIMEOUT_SECONDS);
    }

    /**
     * Creates a dynamic SysId characterization command that applies a step voltage in the requested direction.
     *
     * @param direction sweep direction (forward or reverse)
     * @return command that runs the dynamic test until completion
     */
    public Command createSysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return wrapSysIdCommand(subsystem.getSysIdRoutine().dynamic(direction));
    }

    /**
     * Creates a dynamic SysId command with a timeout to prevent runaway motion.
     *
     * @param direction      sweep direction (forward or reverse)
     * @param timeoutSeconds maximum duration before the command ends
     * @return timed dynamic SysId command
     */
    public Command createSysIdDynamicCommand(SysIdRoutine.Direction direction, double timeoutSeconds) {
        return createSysIdDynamicCommand(direction).withTimeout(timeoutSeconds);
    }

    /**
     * Creates a dynamic SysId command with a default timeout (3 seconds) to safeguard robot motion.
     *
     * @param direction sweep direction (forward or reverse)
     * @return timed dynamic SysId command
     */
    public Command createSysIdDynamicCommandWithDefaultTimeout(SysIdRoutine.Direction direction) {
        return createSysIdDynamicCommand(direction, DEFAULT_SYSID_TIMEOUT_SECONDS);
    }

    /**
     * Creates a full SysId sweep using timing values from the subsystem config.
     * <p>
     * Reads {@code sysIdDelaySeconds}, {@code sysIdQuasistaticTimeoutSeconds}, and {@code sysIdDynamicTimeoutSeconds} from the subsystem's
     * configuration so each mechanism can define its own timing without changing bindings code.
     * </p>
     *
     * @return command that executes the four standard SysId tests in sequence
     */
    public Command createSysIdFullSweepCommand() {
        return createSysIdFullSweepCommand(
                subsystem.getConfig().sysId.getDelaySeconds(),
                subsystem.getConfig().sysId.getQuasistaticTimeoutSeconds(),
                subsystem.getConfig().sysId.getDynamicTimeoutSeconds());
    }

    /**
     * Creates a full SysId sweep (quasistatic forward/reverse, dynamic forward/reverse) with optional delays between phases.
     *
     * @param delaySeconds           pause inserted between each phase to let the mechanism settle
     * @param quasistaticTimeoutSecs timeout for each quasistatic sweep to prevent runaway motion
     * @param dynamicTimeoutSecs     timeout for each dynamic sweep to prevent runaway motion
     * @return command that executes the four standard SysId tests in sequence
     */
    public Command createSysIdFullSweepCommand(double delaySeconds, double quasistaticTimeoutSecs, double dynamicTimeoutSecs) {
        SysIdRoutine routine = subsystem.getSysIdRoutine();

        return wrapSysIdCommand(routine
                .quasistatic(SysIdRoutine.Direction.kForward).withTimeout(quasistaticTimeoutSecs)
                .andThen(Commands.waitSeconds(delaySeconds))
                .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(quasistaticTimeoutSecs))
                .andThen(Commands.waitSeconds(delaySeconds))
                .andThen(routine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(dynamicTimeoutSecs))
                .andThen(Commands.waitSeconds(delaySeconds))
                .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(dynamicTimeoutSecs)));
    }

    /**
     * Builds a command that stops the mechanism immediately by setting velocity to zero.
     *
     * @return command that stops the motor in a single cycle
     */
    public Command createStopCommand() {
        return Commands.runOnce(subsystem::stop, subsystem);
    }

    /**
     * Builds a command that sets the mechanism to a reverse velocity and continuously seeks that speed until interrupted.
     * <p>
     * The target RPM is set once on initialization and then {@code seekVelocity} runs every cycle to maintain the speed. The command never
     * self-finishes, so it persists until the binding releases (e.g., button released) or another command interrupts it. Use this for eject or
     * evacuation behaviors where the motor must keep spinning in reverse for the entire duration a button is held.
     * </p>
     *
     * @param targetRpmSupplier provider for the target RPM (typically negative for reverse); evaluated once on initialize
     * @return command that holds the mechanism at the supplied reverse velocity until interrupted
     */
    public Command createReverseAndHoldCommand(Supplier<Double> targetRpmSupplier) {
        return Commands.runOnce(() -> subsystem.setTargetVelocityRpm(targetRpmSupplier.get()), subsystem)
                .andThen(Commands.run(subsystem::seekVelocity, subsystem))
                .finallyDo(interrupted -> subsystem.stop());
    }

    /**
     * Sets the idle command as the default command for this velocity subsystem.
     * <p>
     * Call this once during {@code RobotContainer} construction so the subsystem returns to its idle behavior whenever no other command is scheduled.
     * </p>
     *
     * @return the idle command that was set as default
     */
    public Command setDefaultIdleCommand() {
        Command command = createIdleCommand();
        subsystem.setDefaultCommand(command);
        return command;
    }

    /**
     * Builds a command that continuously reads a target RPM from a supplier and seeks that velocity every cycle.
     * <p>
     * Unlike commands that lock the target at initialization, this command re-evaluates the supplier each cycle. Use this when the target is backed
     * by a tunable value that operators may adjust while the command is running.
     * </p>
     * <p>
     * When the command ends (button released or interrupted), the subsystem targets zero RPM and runs one final control cycle so the motor,
     * simulation model, and telemetry all reflect the stopped state.
     * </p>
     *
     * @param targetRpmSupplier provider for the target RPM; evaluated every execute cycle
     * @return command that continuously tracks the supplied RPM until interrupted, then stops cleanly
     */
    public Command createContinuousVelocityCommand(Supplier<Double> targetRpmSupplier) {
        // Track the last-applied target so setTargetVelocityRpm is only called when the
        // supplier returns a new value. A small RPM epsilon avoids forwarding trivial
        // floating-point noise that would still pass the base class's rad/s epsilon
        // guard and reset the trapezoidal velocity profile.
        final double TARGET_EPSILON_RPM = 0.1;
        double[]     lastTarget         = { Double.NaN };

        return Commands.run(() -> {
            double currentTarget = targetRpmSupplier.get();
            // Double.NaN comparisons are always false, so the first cycle always applies.
            if (Double.isNaN(lastTarget[0]) || Math.abs(currentTarget - lastTarget[0]) > TARGET_EPSILON_RPM) {
                subsystem.setTargetVelocityRpm(currentTarget);
                lastTarget[0] = currentTarget;
            }
            subsystem.seekVelocity();
        }, subsystem).finallyDo((interrupted) -> {
            // Temporary diagnostic: log why the continuous velocity command ended.
            edu.wpi.first.wpilibj.DriverStation.reportWarning(
                    "ContinuousVelocityCommand ended on " + subsystem.getName()
                            + " interrupted=" + interrupted + " lastTarget=" + lastTarget[0],
                    true);
            lastTarget[0] = Double.NaN;
            subsystem.stop();
        });
    }

    /**
     * Builds the idle command for this velocity subsystem.
     * <p>
     * Each concrete factory returns its own idle command type. The base factory uses this method in {@link #setDefaultIdleCommand()} to wire the
     * subsystem's default behavior.
     * </p>
     *
     * @return command that idles the mechanism at its configured idle RPM
     */
    protected abstract Command createIdleCommand();

    private Command wrapSysIdCommand(Command sysIdCommand) {
        return Commands.sequence(
                Commands.runOnce(subsystem::logSysIdStart, subsystem),
                sysIdCommand)
                .finallyDo(interrupted -> {
                    if (interrupted) {
                        subsystem.logSysIdInterrupted();
                    } else {
                        subsystem.logSysIdEnd();
                    }
                });
    }
}
