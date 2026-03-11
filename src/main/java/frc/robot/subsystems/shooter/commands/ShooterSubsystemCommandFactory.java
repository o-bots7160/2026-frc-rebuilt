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
     * Builds a spin-up command that reads its target RPM from a supplier.
     * <p>
     * Use this for distance-based velocity scaling where the target depends on the robot's position at command start.
     * </p>
     *
     * @param targetRpmSupplier provider for the target flywheel RPM; evaluated on initialize
     * @return command that spins the flywheel to the supplied RPM
     */
    public SpinUpShooterCommand createSpinUpCommand(Supplier<Double> targetRpmSupplier) {
        return new SpinUpShooterCommand(subsystem, targetRpmSupplier);
    }

    /**
     * Builds a spin-up command that drives the flywheel to a fixed RPM.
     *
     * @param targetRpm fixed target flywheel RPM
     * @return command that spins the flywheel to the fixed RPM
     */
    public SpinUpShooterCommand createSpinUpCommand(double targetRpm) {
        return new SpinUpShooterCommand(subsystem, targetRpm);
    }

    /**
     * Builds a command that spins the flywheel to a supplied RPM and then holds that velocity indefinitely.
     * <p>
     * Use this with {@code whileTrue} so the flywheel maintains the target while a button is held and returns to idle when released.
     * </p>
     *
     * @param targetRpmSupplier provider for the target flywheel RPM; evaluated on initialize
     * @return command that spins up and then holds velocity until interrupted
     */
    public Command createSpinUpAndHoldCommand(Supplier<Double> targetRpmSupplier) {
        return Commands.runOnce(() -> subsystem.setTargetVelocityRpm(targetRpmSupplier.get()), subsystem)
                .andThen(Commands.run(subsystem::seekVelocity, subsystem));
    }

    /**
     * Builds a command that spins the flywheel to a fixed RPM and then holds that velocity indefinitely.
     * <p>
     * Use this with {@code whileTrue} so the flywheel maintains the target while a button is held and returns to idle when released.
     * </p>
     *
     * @param targetRpm fixed target flywheel RPM
     * @return command that spins up and then holds velocity until interrupted
     */
    public Command createSpinUpAndHoldCommand(double targetRpm) {
        return createSpinUpAndHoldCommand(() -> targetRpm);
    }

    /**
     * Builds a command that continuously reads a target RPM from a supplier and seeks that velocity every cycle.
     * <p>
     * Unlike {@link #createSpinUpAndHoldCommand(Supplier)}, which locks the target at command start, this command re-evaluates the supplier each
     * cycle. Use this when the target is backed by a tunable value that operators may adjust while the command is running.
     * </p>
     *
     * @param targetRpmSupplier provider for the target flywheel RPM; evaluated every execute cycle
     * @return command that continuously tracks the supplied RPM until interrupted
     */
    public Command createContinuousSpinCommand(Supplier<Double> targetRpmSupplier) {
        return createContinuousVelocityCommand(targetRpmSupplier);
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
}
