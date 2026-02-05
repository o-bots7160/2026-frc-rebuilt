package frc.robot.subsystems.turret.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shared.commands.AbstractSetAndSeekCommandFactory;
import frc.robot.subsystems.robotstate.RobotStateSubsystem;
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
    
    /**
     * Builds a command that continuously tracks a field-relative target using the fused robot pose.
     *
     * @param robotStateSubsystem         robot state subsystem providing the fused pose estimate
     * @param targetFieldPositionSupplier supplier of the target position in field coordinates (meters)
     * @return command that aims the turret at the supplied field target position
     */
    public TrackFieldTargetCommand createTrackFieldTargetCommand(
            RobotStateSubsystem robotStateSubsystem,
            Supplier<Translation2d> targetFieldPositionSupplier) {
        return new TrackFieldTargetCommand(subsystem, robotStateSubsystem, targetFieldPositionSupplier);
    }

    /**
     * Builds and sets the default turret tracking command using a field-relative target supplier.
     *
     * @param robotStateSubsystem         robot state subsystem providing the fused pose estimate
     * @param targetFieldPositionSupplier supplier of the target position in field coordinates (meters)
     * @return command that is also set as the turret default
     */
    public Command setDefaultTrackFieldTargetCommand(
            RobotStateSubsystem robotStateSubsystem,
            Supplier<Translation2d> targetFieldPositionSupplier) {
        Command command = createTrackFieldTargetCommand(robotStateSubsystem, targetFieldPositionSupplier);
        subsystem.setDefaultCommand(command);
        return command;
    }
}
