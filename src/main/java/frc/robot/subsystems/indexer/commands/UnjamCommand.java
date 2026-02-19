package frc.robot.subsystems.indexer.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.shared.commands.AbstractSubsystemCommand;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indexer.config.IndexerSubsystemConfig;

/**
 * Command that alternates between forward and reverse pulses to dislodge a jammed piece of Fuel.
 * <p>
 * Each cycle spins the roller forward for the configured forward duration, then reverses for the configured reverse duration. The alternating motion
 * works the stuck piece loose without requiring the operator to manually coordinate direction changes. The command runs until interrupted by the
 * operator.
 * </p>
 */
public class UnjamCommand extends AbstractSubsystemCommand<IndexerSubsystem> {

    private final IndexerSubsystemConfig config;

    private final Timer                  cycleTimer = new Timer();

    /** True when the current pulse is forward; false when reverse. */
    private boolean                      isForwardPhase;

    /**
     * Creates an unjam command for the indexer.
     *
     * @param subsystem indexer subsystem to pulse back and forth
     * @param config    indexer configuration supplying feed RPM, reverse RPM, and pulse durations
     */
    public UnjamCommand(IndexerSubsystem subsystem, IndexerSubsystemConfig config) {
        super(subsystem);
        this.config = config;
    }

    @Override
    public void execute() {
        subsystem.seekVelocity();

        double currentPhaseDuration = isForwardPhase
                ? config.getUnjamForwardDurationSeconds()
                : config.getUnjamReverseDurationSeconds();

        if (cycleTimer.hasElapsed(currentPhaseDuration)) {
            // Switch direction.
            isForwardPhase = !isForwardPhase;
            if (isForwardPhase) {
                subsystem.setFeedVelocity();
            } else {
                subsystem.setReverseVelocity();
            }
            cycleTimer.restart();
        }
    }

    @Override
    public void end(boolean interrupted) {
        cycleTimer.stop();
        subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // Runs until interrupted by the operator.
        return false;
    }

    @Override
    protected void onInitialize() {
        isForwardPhase = true;
        cycleTimer.restart();
        subsystem.setFeedVelocity();
    }
}
