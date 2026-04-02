package frc.robot.subsystems.indexer.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.commands.AbstractVelocityCommandFactory;
import frc.robot.subsystems.indexer.IndexerSubsystem;

/**
 * Factory that creates indexer commands and wires default behaviors.
 */
public class IndexerSubsystemCommandFactory extends AbstractVelocityCommandFactory<IndexerSubsystem> {

    /**
     * Creates a factory for commands that share the given indexer subsystem instance.
     *
     * @param subsystem indexer subsystem instance that commands created by this factory will control
     */
    public IndexerSubsystemCommandFactory(IndexerSubsystem subsystem) {
        super(subsystem);
    }

    /**
     * Builds an idle command that holds the roller at the configured idle RPM (typically 0).
     *
     * @return command that idles the indexer roller
     */
    @Override
    public IdleIndexerCommand createIdleCommand() {
        return new IdleIndexerCommand(subsystem);
    }

    /**
     * Builds a reverse command that reads its target RPM from a supplier.
     *
     * @param targetRpmSupplier provider for the target reverse RPM (negative value); evaluated on initialize
     * @return command that spins the roller in reverse at the supplied RPM
     */
    public ReverseIndexerCommand createReverseCommand(Supplier<Double> targetRpmSupplier) {
        return new ReverseIndexerCommand(subsystem, targetRpmSupplier);
    }

    /**
     * Builds a reverse command using the configured default reverse velocity.
     * <p>
     * The configured reverse RPM is negated so the roller spins backward toward the feeder.
     * </p>
     *
     * @return command that spins the roller in reverse at the default RPM
     */
    public ReverseIndexerCommand createReverseCommand() {
        return new ReverseIndexerCommand(subsystem, -subsystem.getConfig().getReverseVelocityRpm());
    }

    /**
     * Builds a composite command that waits until the shooter and turret are both ready, then feeds Fuel into the shooter.
     * <p>
     * The command holds the indexer stopped while either readiness supplier returns false. Once both report true, it feeds forward at the configured
     * feed velocity. This keeps the indexer decoupled from the shooter and turret subsystems — readiness is checked through suppliers wired in
     * {@code RobotContainer}.
     * </p>
     *
     * @param shooterReadySupplier   supplier that returns true when the shooter flywheel is at the target RPM
     * @param turretOnTargetSupplier supplier that returns true when the turret is aimed at the scoring target
     * @return composite command that waits for readiness then feeds
     */
    public Command createFireWhenReadyCommand(Supplier<Boolean> shooterReadySupplier, Supplier<Boolean> turretOnTargetSupplier) {
        Supplier<Boolean> readySupplier = () -> shooterReadySupplier.get() && turretOnTargetSupplier.get();
        return Commands.run(() -> {
                    if (readySupplier.get()) {
                        subsystem.setTargetVelocityRpm(subsystem.getConfig().getFeedVelocityRpm());
                    } else {
                        subsystem.setTargetVelocityRpm(subsystem.getConfig().motionProfile.getIdleVelocityRpm());
                    }
                    subsystem.seekVelocity();
                }, subsystem)
                .finallyDo(() -> subsystem.stop())
                .withName("FireWhenReady");
    }

}
