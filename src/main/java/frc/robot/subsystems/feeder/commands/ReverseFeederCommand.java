package frc.robot.subsystems.feeder.commands;

import java.util.function.Supplier;

import frc.robot.shared.commands.AbstractVelocityCommand;
import frc.robot.subsystems.feeder.FeederSubsystem;

/**
 * Command that spins the feeder belt in reverse to clear Fuel back toward the intake.
 * <p>
 * The target RPM supplier is evaluated once on initialize. Pass a negative RPM to reverse the belt direction. The command finishes when the belt
 * reaches the target velocity and has been stable for the configured settle time. Bind this with {@code whileTrue} so the belt returns to its default
 * forward idle when the operator releases the button.
 * </p>
 */
public class ReverseFeederCommand extends AbstractVelocityCommand<FeederSubsystem> {

    /**
     * Creates a reverse command that reads its target RPM from a supplier.
     *
     * @param subsystem         feeder subsystem to control
     * @param targetRpmSupplier provider for the target reverse RPM (negative value); evaluated on initialize
     */
    public ReverseFeederCommand(FeederSubsystem subsystem, Supplier<Double> targetRpmSupplier) {
        super(subsystem, targetRpmSupplier);
    }

    /**
     * Creates a reverse command that drives the belt at a fixed negative RPM.
     *
     * @param subsystem feeder subsystem to control
     * @param targetRpm fixed target reverse RPM (negative value for backward rotation)
     */
    public ReverseFeederCommand(FeederSubsystem subsystem, double targetRpm) {
        this(subsystem, () -> targetRpm);
    }
}
