package frc.robot.subsystems.harvester.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shared.commands.AbstractSetAndSeekCommandFactory;
import frc.robot.subsystems.harvester.HarvesterSubsystem;

/**
 * Factory that creates harvester arm commands and wires default behaviors.
 */
public class HarvesterSubsystemCommandFactory extends AbstractSetAndSeekCommandFactory<HarvesterSubsystem> {

    /**
     * Creates a factory for commands that share the given harvester subsystem instance.
     *
     * @param subsystem harvester subsystem instance that commands created by this factory will control
     */
    public HarvesterSubsystemCommandFactory(HarvesterSubsystem subsystem) {
        super(subsystem);
    }

    /**
     * Builds a profiled move command that reads its target angle from a supplier.
     *
     * @param targetDegreesSupplier supplier providing the desired arm angle in degrees
     * @return command that drives the arm toward the supplied target
     */
    public MoveHarvesterToPositionCommand createMoveToPositionCommand(Supplier<Double> targetDegreesSupplier) {
        return new MoveHarvesterToPositionCommand(subsystem, targetDegreesSupplier);
    }

    /**
     * Builds a profiled move command that drives the arm to a fixed angle.
     *
     * @param targetDegrees desired arm angle in degrees
     * @return command that drives the arm to the fixed target
     */
    public MoveHarvesterToPositionCommand createMoveToPositionCommand(double targetDegrees) {
        return createMoveToPositionCommand(() -> targetDegrees);
    }

    /**
     * Builds a command that deploys the arm to the lowered Fuel-collection position.
     *
     * @return command that moves the arm to the deployed angle
     */
    public MoveHarvesterToPositionCommand createDeployCommand() {
        return createMoveToPositionCommand(subsystem.getConfig()::getDeployedPositionDegrees);
    }

    /**
     * Builds a command that stows the arm in the upright match-start position.
     *
     * @return command that moves the arm to the stowed angle
     */
    public MoveHarvesterToPositionCommand createStowCommand() {
        return createMoveToPositionCommand(subsystem.getConfig()::getStowedPositionDegrees);
    }

    /**
     * Builds a stow command and sets it as the default command for the harvester subsystem.
     * <p>
     * When no other command is scheduled, the arm will seek the stowed position so it stays inside the robot perimeter.
     * </p>
     *
     * @return the stow command that was set as the default
     */
    public Command setDefaultStowCommand() {
        Command command = createStowCommand();
        subsystem.setDefaultCommand(command);
        return command;
    }

    /**
     * Builds a hold command that monitors the deployed position and re-engages the motor when the arm is pushed away by incoming Fuel.
     *
     * @return command that holds the deployed position when active and idles otherwise
     */
    public HoldHarvesterDeployedPositionCommand createHoldDeployedPositionCommand() {
        return new HoldHarvesterDeployedPositionCommand(subsystem);
    }

    /**
     * Builds a hold command and sets it as the default command for the harvester subsystem.
     * <p>
     * When no other command is scheduled, the arm will monitor for drift from the deployed position and re-engage the motor to correct it. When the
     * arm is not deployed, the command idles passively.
     * </p>
     *
     * @return the hold command that was set as the default
     */
    public Command setDefaultHoldDeployedPositionCommand() {
        Command command = createHoldDeployedPositionCommand();
        subsystem.setDefaultCommand(command);
        return command;
    }
}
