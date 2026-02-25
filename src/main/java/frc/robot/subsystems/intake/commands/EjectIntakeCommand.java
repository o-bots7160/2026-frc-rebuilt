package frc.robot.subsystems.intake.commands;

import java.util.function.Supplier;

import frc.robot.shared.commands.AbstractVelocityCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Command that spins the intake rollers in reverse to eject Fuel back onto the field.
 * <p>
 * The target RPM supplier is evaluated once on initialize. Pass a negative RPM to reverse the roller direction. The command finishes when the rollers
 * reach the target velocity and have been stable for the configured settle time. Bind this with {@code whileTrue} so the rollers return to their
 * default idle when the operator releases the button.
 * </p>
 */
public class EjectIntakeCommand extends AbstractVelocityCommand<IntakeSubsystem> {

    /**
     * Creates an eject command that reads its target RPM from a supplier.
     *
     * @param subsystem         intake subsystem to control
     * @param targetRpmSupplier provider for the target reverse RPM (negative value); evaluated on initialize
     */
    public EjectIntakeCommand(IntakeSubsystem subsystem, Supplier<Double> targetRpmSupplier) {
        super(subsystem, targetRpmSupplier);
    }

    /**
     * Creates an eject command that drives the rollers at a fixed negative RPM.
     *
     * @param subsystem intake subsystem to control
     * @param targetRpm fixed target reverse RPM (negative value for backward rotation)
     */
    public EjectIntakeCommand(IntakeSubsystem subsystem, double targetRpm) {
        this(subsystem, () -> targetRpm);
    }
}
