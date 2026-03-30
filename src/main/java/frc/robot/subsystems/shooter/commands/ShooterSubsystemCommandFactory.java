package frc.robot.subsystems.shooter.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shared.commands.AbstractVelocityCommandFactory;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Generates commands that operate on the shooter subsystem so RobotContainer can stay focused on wiring.
 */
public class ShooterSubsystemCommandFactory extends AbstractVelocityCommandFactory<ShooterSubsystem> {

    /**
     * Creates a factory for commands that share the given shooter subsystem instance.
     *
     * @param subsystem shooter subsystem instance that commands created by this factory will control
     */
    public ShooterSubsystemCommandFactory(ShooterSubsystem subsystem) {
        super(subsystem);
    }

    /**
     * Builds an idle command that holds the flywheel at the configured idle RPM.
     *
     * @return command that idles the shooter flywheel
     */
    @Override
    public IdleShooterCommand createIdleCommand() {
        return new IdleShooterCommand(subsystem);
    }

    /**
     * Builds a command that continuously computes flywheel RPM from the robot's distance to a target.
     * <p>
     * Each cycle, the command reads the current distance from the supplier, converts it to an RPM via the subsystem's interpolation table, and seeks
     * that velocity. Use this as a default command or bind it to a button so the shooter automatically adjusts speed as the robot moves.
     * </p>
     *
     * @param distanceMetersSupplier supplier returning the distance from the robot to the active target in meters
     * @return command that continuously tracks distance-based RPM until interrupted
     */
    public Command createDistanceBasedSpinCommand(Supplier<Double> distanceMetersSupplier) {
        return createContinuousSpinCommand(() -> subsystem.calculateRpmFromDistanceMeters(distanceMetersSupplier.get()));
    }

    /**
     * Builds a command that boosts the shooter RPM by the configured adjustment amount while held.
     * <p>
     * The command does not require the shooter subsystem so it can run concurrently with an active shooting command. When the command ends (trigger
     * released), the offset resets to zero.
     * </p>
     *
     * @return command that applies a positive RPM offset while active
     */
    public Command createBoostRpmCommand() {
        return Commands.startEnd(
                () -> subsystem.setRpmOffset(subsystem.getConfig().getRpmAdjustmentAmountRpm()),
                () -> subsystem.setRpmOffset(0.0))
                .withName("Shooter-BoostRpm");
    }

    /**
     * Builds a command that cuts the shooter RPM by the configured adjustment amount while held.
     * <p>
     * The command does not require the shooter subsystem so it can run concurrently with an active shooting command. When the command ends (trigger
     * released), the offset resets to zero.
     * </p>
     *
     * @return command that applies a negative RPM offset while active
     */
    public Command createCutRpmCommand() {
        return Commands.startEnd(
                () -> subsystem.setRpmOffset(-subsystem.getConfig().getRpmAdjustmentAmountRpm()),
                () -> subsystem.setRpmOffset(0.0))
                .withName("Shooter-CutRpm");
    }

    /**
     * Builds a command that continuously reads a target RPM from a supplier and seeks that velocity every cycle.
     *
     * @param targetRpmSupplier provider for the target flywheel RPM; evaluated every execute cycle
     * @return command that continuously tracks the supplied RPM until interrupted
     */
    private Command createContinuousSpinCommand(Supplier<Double> targetRpmSupplier) {
        return createContinuousVelocityCommand(targetRpmSupplier);
    }
}
