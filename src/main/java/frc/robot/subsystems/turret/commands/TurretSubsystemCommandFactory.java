package frc.robot.subsystems.turret.commands;

import java.util.function.Supplier;

import frc.robot.shared.commands.AbstractSetAndSeekCommandFactory;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * Generates commands that operate on the turret subsystem so RobotContainer can stay focused on wiring.
 */
public class TurretSubsystemCommandFactory extends AbstractSetAndSeekCommandFactory<TurretSubsystem> {

    /**
     * Creates a factory for commands that share the given turret subsystem instance.
     *
     * @param subsystem turret subsystem instance that commands created by this factory will control
     */
    public TurretSubsystemCommandFactory(TurretSubsystem subsystem) {
        super(subsystem);
    }

    /**
     * Builds a profiled move command that reads its target angle from a supplier.
     *
     * @param targetDegreesSupplier supplier providing the desired turret angle in degrees
     * @return command that drives the turret toward the supplied target
     */
    public MoveTurretToAngleCommand createMoveToAngleCommand(Supplier<Double> targetDegreesSupplier) {
        return new MoveTurretToAngleCommand(subsystem, targetDegreesSupplier);
    }

    /**
     * Builds a profiled move command that drives the turret to a fixed angle.
     *
     * @param targetDegrees desired turret angle in degrees
     * @return command that drives the turret to the fixed target
     */
    public MoveTurretToAngleCommand createMoveToAngleCommand(double targetDegrees) {
        return createMoveToAngleCommand(() -> targetDegrees);
    }
}
