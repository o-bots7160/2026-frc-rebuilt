package frc.robot.subsystems.climber.commands;

import java.util.function.Supplier;

import frc.robot.shared.commands.AbstractSetAndSeekCommandFactory;
import frc.robot.subsystems.climber.ClimberSubsystem;

/**
 * Generates commands that operate on the climber subsystem so RobotContainer can stay focused on wiring.
 * <p>
 * Move-to-position commands are placeholders until motor hardware is selected. Additional climb-sequence and manual-override commands will be added
 * once the motor implementation is complete.
 * </p>
 */
public class ClimberSubsystemCommandFactory extends AbstractSetAndSeekCommandFactory<ClimberSubsystem> {

    /**
     * Creates a factory that produces commands operating on the provided climber subsystem.
     *
     * @param subsystem the climber subsystem that all produced commands will require
     */
    public ClimberSubsystemCommandFactory(ClimberSubsystem subsystem) {
        super(subsystem);
    }

    /**
     * Builds a command that drives the climber to the position returned by the supplier.
     *
     * @param targetDegreesSupplier supplier that returns the target position in degrees
     * @return command that moves the climber to the supplied position
     */
    public MoveClimberToPositionCommand createMoveToPositionCommand(Supplier<Double> targetDegreesSupplier) {
        return new MoveClimberToPositionCommand(subsystem, targetDegreesSupplier);
    }

    /**
     * Builds a command that drives the climber to a fixed position in degrees.
     *
     * @param targetDegrees target position in degrees
     * @return command that moves the climber to the specified position
     */
    public MoveClimberToPositionCommand createMoveToPositionCommand(double targetDegrees) {
        return createMoveToPositionCommand(() -> targetDegrees);
    }
}
