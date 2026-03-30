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
     * The supplier should return a negative RPM so the belt spins backward toward the intake.
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
     * The configured reverse RPM is negated so the belt spins backward toward the intake.
     * </p>
     *
     * @return command that spins the belt in reverse at the default clearing RPM
     */
    public ReverseFeederCommand createReverseCommand() {
        return new ReverseFeederCommand(subsystem, -subsystem.getConfig().getReverseVelocityRpm());
    }

    /**
     * Builds a composite command that waits until the shooter and turret are both ready, then runs the reverse-pulse-then-forward sequence.
     * <p>
     * The command blocks while either readiness supplier returns false. Once both report true, it runs a short reverse pulse (dislodging jammed Fuel)
     * and then transports forward. This keeps the feeder decoupled from the shooter and turret subsystems — readiness is checked through suppliers
     * wired in {@code RobotContainer}.
     * </p>
     *
     * @param shooterReadySupplier   supplier that returns true when the shooter flywheel is at the target RPM
     * @param turretOnTargetSupplier supplier that returns true when the turret is aimed at the scoring target
     * @return composite command that waits for readiness then runs the reverse-pulse-then-forward sequence
     */
    public Command createFireWhenReadyCommand(Supplier<Boolean> shooterReadySupplier, Supplier<Boolean> turretOnTargetSupplier) {
        Supplier<Boolean> readySupplier = () -> shooterReadySupplier.get() && turretOnTargetSupplier.get();
        return Commands.waitUntil(readySupplier::get)
                .andThen(
                        // Run the reverse pulse to completion so the belt always transitions
                        // to a forward target before readiness can interrupt the sequence.
                        createReversePulseCommand(),
                        createForwardAndHoldCommand()
                                .onlyWhile(readySupplier::get))
                .repeatedly()
                .finallyDo(() -> subsystem.stop())
                .withName("FeederFireWhenReady");
    }

    /**
     * Builds a command that spins the belt forward at the configured transport velocity and holds that speed indefinitely.
     *
     * @return command that transports Fuel forward and holds velocity until interrupted
     */
    private Command createForwardAndHoldCommand() {
        return Commands.runOnce(subsystem::setForwardVelocity, subsystem)
                .andThen(Commands.run(subsystem::seekVelocity, subsystem));
    }

    /**
     * Builds a short reverse pulse command that sets the belt to reverse velocity and seeks for the configured pulse duration.
     * <p>
     * The pulse always runs to completion and is not gated by readiness suppliers. This prevents the belt from getting stuck in reverse when
     * readiness flickers during the pulse window.
     * </p>
     *
     * @return command that reverses the belt for the configured pulse duration
     */
    private Command createReversePulseCommand() {
        return Commands.runOnce(subsystem::setReverseVelocity, subsystem)
                .andThen(Commands.run(subsystem::seekVelocity, subsystem)
                        .withTimeout(subsystem.getConfig().getReversePulseDurationSeconds()))
                .withName("FeederReversePulse");
    }
}
