package frc.robot.subsystems.hopper.commands;

import java.util.function.Supplier;

import frc.robot.shared.commands.AbstractVelocityCommand;
import frc.robot.subsystems.hopper.HopperSubsystem;

/**
 * Command that spins the hopper belt in reverse to purge Fuel back through the intake.
 * <p>
 * The target RPM supplier is evaluated once on initialize. Pass a negative RPM to reverse the belt direction. The command finishes when the belt
 * reaches the target velocity and has been stable for the configured settle time. Bind this with {@code whileTrue} so the belt returns to its default
 * forward idle when the operator releases the button.
 * </p>
 */
public class PurgeHopperCommand extends AbstractVelocityCommand<HopperSubsystem> {

    /**
     * Creates a purge command that reads its target RPM from a supplier.
     *
     * @param subsystem         hopper subsystem to control
     * @param targetRpmSupplier provider for the target reverse RPM (negative value); evaluated on initialize
     */
    public PurgeHopperCommand(HopperSubsystem subsystem, Supplier<Double> targetRpmSupplier) {
        super(subsystem, targetRpmSupplier);
    }

    /**
     * Creates a purge command that drives the belt at a fixed negative RPM.
     *
     * @param subsystem hopper subsystem to control
     * @param targetRpm fixed target reverse RPM (negative value for backward rotation)
     */
    public PurgeHopperCommand(HopperSubsystem subsystem, double targetRpm) {
        this(subsystem, () -> targetRpm);
    }
}
