package frc.robot.subsystems.intake.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.commands.AbstractVelocityCommandFactory;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Factory that creates intake commands and wires default behaviors.
 */
public class IntakeSubsystemCommandFactory extends AbstractVelocityCommandFactory<IntakeSubsystem> {

    /**
     * Creates a factory for commands that share the given intake subsystem instance.
     *
     * @param subsystem intake subsystem instance that commands created by this factory will control
     */
    public IntakeSubsystemCommandFactory(IntakeSubsystem subsystem) {
        super(subsystem);
    }

    /**
     * Builds an idle command that holds the rollers at the configured idle RPM (typically zero).
     *
     * @return command that idles the intake rollers
     */
    @Override
    public IdleIntakeCommand createIdleCommand() {
        return new IdleIntakeCommand(subsystem);
    }

    /**
     * Builds an eject command that reads its target RPM from a supplier.
     * <p>
     * The supplier should return a negative RPM so the rollers spin backward to push Fuel out.
     * </p>
     *
     * @param targetRpmSupplier provider for the target reverse RPM (negative value); evaluated on initialize
     * @return command that spins the rollers in reverse at the supplied RPM
     */
    public EjectIntakeCommand createEjectCommand(Supplier<Double> targetRpmSupplier) {
        return new EjectIntakeCommand(subsystem, targetRpmSupplier);
    }

    /**
     * Builds an eject command using the configured default reverse velocity.
     * <p>
     * The configured reverse RPM is negated so the rollers spin backward to push Fuel out.
     * </p>
     *
     * @return command that spins the rollers in reverse at the default eject RPM
     */
    public EjectIntakeCommand createEjectCommand() {
        return new EjectIntakeCommand(subsystem, -subsystem.getConfig().getReverseVelocityRpm());
    }

    /**
     * Builds a command that spins the rollers forward at the configured intake velocity and holds that speed indefinitely.
     * <p>
     * Use this with {@code whileTrue} so the rollers pull Fuel while a button is held and return to idle when released.
     * </p>
     *
     * @return command that intakes Fuel and holds velocity until interrupted
     */
    public Command createIntakeAndHoldCommand() {
        return Commands.runOnce(subsystem::setForwardVelocity, subsystem)
                .andThen(Commands.run(subsystem::seekVelocity, subsystem));
    }

    /**
     * Builds a command that spins the rollers in reverse at the configured eject velocity and holds that speed indefinitely.
     * <p>
     * Use this with {@code whileTrue} so the rollers eject Fuel while a button is held and return to idle when released.
     * </p>
     *
     * @return command that ejects Fuel backward and holds velocity until interrupted
     */
    public Command createEjectAndHoldCommand() {
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
     * @param targetRpmSupplier provider for the target roller RPM; evaluated every execute cycle
     * @return command that continuously tracks the supplied RPM until interrupted
     */
    public Command createContinuousIntakeCommand(Supplier<Double> targetRpmSupplier) {
        return createContinuousVelocityCommand(targetRpmSupplier);
    }
}
