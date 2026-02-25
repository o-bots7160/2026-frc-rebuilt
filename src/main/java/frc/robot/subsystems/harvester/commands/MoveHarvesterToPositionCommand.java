package frc.robot.subsystems.harvester.commands;

import java.util.function.Supplier;

import frc.robot.shared.commands.AbstractSetAndSeekCommand;
import frc.robot.subsystems.harvester.HarvesterSubsystem;

/**
 * Drives the harvester arm toward a target angle using its trapezoidal profile. Use this for preset positions (stow/deploy) or variable targets
 * supplied at runtime.
 */
public class MoveHarvesterToPositionCommand extends AbstractSetAndSeekCommand<HarvesterSubsystem> {

    /**
     * Builds a profiled harvester move command that reads the target angle from a supplier.
     *
     * @param harvesterSubsystem    harvester subsystem to control
     * @param targetDegreesSupplier supplier providing the desired arm angle in degrees
     */
    public MoveHarvesterToPositionCommand(HarvesterSubsystem harvesterSubsystem, Supplier<Double> targetDegreesSupplier) {
        super(harvesterSubsystem, targetDegreesSupplier);
    }

    /**
     * Builds a profiled harvester move command that drives to a fixed angle.
     *
     * @param harvesterSubsystem harvester subsystem to control
     * @param targetDegrees      desired arm angle in degrees
     */
    public MoveHarvesterToPositionCommand(HarvesterSubsystem harvesterSubsystem, double targetDegrees) {
        this(harvesterSubsystem, () -> targetDegrees);
    }
}
