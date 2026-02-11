package frc.robot.subsystems.shooter.commands;

import java.util.function.Supplier;

import frc.robot.shared.commands.AbstractVelocityCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Command that spins the shooter flywheel up to a target RPM provided by a supplier.
 * <p>
 * The target RPM supplier is evaluated once on initialize. Use this for distance-based velocity scaling where the target is computed from the robot's
 * position on the field. The command finishes when the flywheel reaches the target velocity and has been stable for the configured settle time.
 * </p>
 */
public class SpinUpShooterCommand extends AbstractVelocityCommand<ShooterSubsystem> {

    /**
     * Creates a spin-up command that reads its target RPM from a supplier.
     *
     * @param subsystem         shooter subsystem to control
     * @param targetRpmSupplier provider for the target flywheel RPM; evaluated on initialize
     */
    public SpinUpShooterCommand(ShooterSubsystem subsystem, Supplier<Double> targetRpmSupplier) {
        super(subsystem, targetRpmSupplier);
    }

    /**
     * Creates a spin-up command that drives the flywheel to a fixed RPM.
     *
     * @param subsystem shooter subsystem to control
     * @param targetRpm fixed target flywheel RPM
     */
    public SpinUpShooterCommand(ShooterSubsystem subsystem, double targetRpm) {
        this(subsystem, () -> targetRpm);
    }
}
