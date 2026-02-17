package frc.robot.subsystems.climber.commands;

import java.util.function.Supplier;

import frc.robot.shared.commands.AbstractSetAndSeekCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;

/**
 * Drives the climber toward a target position using its trapezoidal motion profile. Use this for preset stage positions during the endgame climb
 * sequence.
 * <p>
 * This command is a placeholder that inherits all behavior from {@link AbstractSetAndSeekCommand}. It will become functional once motor hardware is
 * selected and wired into the climber subsystem.
 * </p>
 */
public class MoveClimberToPositionCommand extends AbstractSetAndSeekCommand<ClimberSubsystem> {

    /**
     * Creates a move command that reads its target from a supplier each time it is scheduled.
     *
     * @param climberSubsystem      the climber subsystem to drive
     * @param targetDegreesSupplier supplier that returns the target position in degrees
     */
    public MoveClimberToPositionCommand(ClimberSubsystem climberSubsystem, Supplier<Double> targetDegreesSupplier) {
        super(climberSubsystem, targetDegreesSupplier);
    }

    /**
     * Creates a move command with a fixed target position.
     *
     * @param climberSubsystem the climber subsystem to drive
     * @param targetDegrees    target position in degrees
     */
    public MoveClimberToPositionCommand(ClimberSubsystem climberSubsystem, double targetDegrees) {
        this(climberSubsystem, () -> targetDegrees);
    }
}
