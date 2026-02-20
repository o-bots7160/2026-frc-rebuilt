package frc.robot.subsystems.feeder.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.commands.AbstractVelocityCommandFactory;
import frc.robot.subsystems.feeder.FeederSubsystem;

/**
 * Factory that creates feeder commands and wires default behaviors.
 */
public class FeederSubsystemCommandFactory extends AbstractVelocityCommandFactory<FeederSubsystem> {

    /**
     * Creates a factory for commands that share the given feeder subsystem instance.
     *
     * @param subsystem feeder subsystem instance that commands created by this factory will control
     */
    public FeederSubsystemCommandFactory(FeederSubsystem subsystem) {
        super(subsystem);
    }

    /**
     * Builds an idle command that holds the belt at the configured idle RPM.
     *
     * @return command that idles the feeder belt
     */
    @Override
    public IdleFeederCommand createIdleCommand() {
        return new IdleFeederCommand(subsystem);
    }

    /**
     * Builds a reverse command that reads its target RPM from a supplier.
     * <p>
     * The supplier should return a negative RPM so the belt spins backward toward the hopper.
     * </p>
     *
     * @param targetRpmSupplier provider for the target reverse RPM (negative value); evaluated on initialize
     * @return command that spins the belt in reverse at the supplied RPM
     */
    public ReverseFeederCommand createReverseCommand(Supplier<Double> targetRpmSupplier) {
        return new ReverseFeederCommand(subsystem, targetRpmSupplier);
    }

    /**
     * Builds a reverse command using the configured default reverse velocity.
     * <p>
     * The configured reverse RPM is negated so the belt spins backward toward the hopper.
     * </p>
     *
     * @return command that spins the belt in reverse at the default clearing RPM
     */
    public ReverseFeederCommand createReverseCommand() {
        return new ReverseFeederCommand(subsystem, -subsystem.getConfig().getReverseVelocityRpm());
    }

    /**
     * Builds a command that spins the belt forward at the configured transport velocity and holds that speed indefinitely.
     * <p>
     * Use this with {@code whileTrue} so the belt transports Fuel while a button is held and returns to idle when released.
     * </p>
     *
     * @return command that transports Fuel forward and holds velocity until interrupted
     */
    public Command createForwardAndHoldCommand() {
        return Commands.runOnce(subsystem::setForwardVelocity, subsystem)
                .andThen(Commands.run(subsystem::seekVelocity, subsystem));
    }

    /**
     * Builds a command that spins the belt in reverse at the configured clearing velocity and holds that speed indefinitely.
     * <p>
     * Use this with {@code whileTrue} so the belt clears Fuel while a button is held and returns to idle when released.
     * </p>
     *
     * @return command that clears Fuel backward and holds velocity until interrupted
     */
    public Command createReverseAndHoldCommand() {
        return Commands.runOnce(subsystem::setReverseVelocity, subsystem)
                .andThen(Commands.run(subsystem::seekVelocity, subsystem));
    }

    /**
     * Builds a command that continuously reads a target RPM from a supplier and seeks that velocity every cycle.
     * <p>
     * Unlike fixed-target commands which lock the target at command start, this command re-evaluates the supplier each cycle. Use this when the
     * target is backed by a tunable value that operators may adjust while the command is running.
     * </p>
     *
     * @param targetRpmSupplier provider for the target belt RPM; evaluated every execute cycle
     * @return command that continuously tracks the supplied RPM until interrupted
     */
    public Command createContinuousTransportCommand(Supplier<Double> targetRpmSupplier) {
        return createContinuousVelocityCommand(targetRpmSupplier);
    }
}
