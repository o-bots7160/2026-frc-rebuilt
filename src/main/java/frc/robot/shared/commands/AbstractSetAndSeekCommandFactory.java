package frc.robot.shared.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.shared.subsystems.AbstractSetAndSeekSubsystem;

/**
 * Command factory for set-and-seek style subsystems that exposes common SysId characterization commands while keeping subsystems free of command
 * creation.
 */
public class AbstractSetAndSeekCommandFactory<TSubsystem extends AbstractSetAndSeekSubsystem<?>>
        extends AbstractSubsystemCommandFactory<TSubsystem> {

    private static final double DEFAULT_SYSID_TIMEOUT_SECONDS = 3.0;

    /**
     * Builds a command factory bound to a set-and-seek subsystem.
     *
     * @param subsystem set-and-seek subsystem instance shared by generated commands
     */
    protected AbstractSetAndSeekCommandFactory(TSubsystem subsystem) {
        super(subsystem);
    }

    /**
     * Creates a quasistatic SysId characterization command that ramps voltage slowly in the requested direction.
     *
     * @param direction sweep direction (forward or reverse)
     * @return command that runs the quasistatic test until completion
     */
    public Command createSysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return subsystem.getSysIdRoutine().quasistatic(direction);
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
        return subsystem.getSysIdRoutine().dynamic(direction);
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
     * Creates a full SysId sweep (quasistatic forward/reverse, dynamic forward/reverse) with optional delays between phases.
     *
     * @param delaySeconds           pause inserted between each phase to let the mechanism settle
     * @param quasistaticTimeoutSecs timeout for each quasistatic sweep to prevent runaway motion
     * @param dynamicTimeoutSecs     timeout for each dynamic sweep to prevent runaway motion
     * @return command that executes the four standard SysId tests in sequence
     */
    public Command createSysIdFullSweepCommand(double delaySeconds, double quasistaticTimeoutSecs, double dynamicTimeoutSecs) {
        SysIdRoutine routine = subsystem.getSysIdRoutine();

        return routine
                .quasistatic(SysIdRoutine.Direction.kForward)
                .withTimeout(quasistaticTimeoutSecs)
                .andThen(Commands.waitSeconds(delaySeconds))
                .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(quasistaticTimeoutSecs))
                .andThen(Commands.waitSeconds(delaySeconds))
                .andThen(routine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(dynamicTimeoutSecs))
                .andThen(Commands.waitSeconds(delaySeconds))
                .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(dynamicTimeoutSecs));
    }
}
