package frc.robot.shared.bindings;

import frc.robot.subsystems.drivebase.commands.DriveBaseSubsystemCommandFactory;
import frc.robot.subsystems.feeder.commands.FeederSubsystemCommandFactory;
import frc.robot.subsystems.gameplaystate.commands.GameplayStateCommandFactory;
import frc.robot.subsystems.harvester.commands.HarvesterSubsystemCommandFactory;
import frc.robot.subsystems.indexer.commands.IndexerSubsystemCommandFactory;
import frc.robot.subsystems.intake.commands.IntakeSubsystemCommandFactory;
import frc.robot.subsystems.shooter.commands.ShooterSubsystemCommandFactory;
import frc.robot.subsystems.turret.commands.TurretSubsystemCommandFactory;

/**
 * Competition binding strategy that wires driver and operator controllers to gameplay commands. The driver controls field-relative driving, speed
 * tiers, heading snaps, wheel lock, and d-pad pathfinding. The operator manages gameplay state transitions, shooter RPM adjustments, and turret lock.
 */
public class CompetitionTriggerBindings extends AbstractTriggerBindings {

    /**
     * Creates competition trigger bindings with the default controller ports.
     *
     * @param driveBaseCommandFactory     factory for creating drive base commands
     * @param triggerBindingsConfig       configuration for per-axis response curves and speed tiers
     * @param turretCommandFactory        factory for creating turret commands
     * @param shooterCommandFactory       factory for creating shooter commands
     * @param indexerCommandFactory       factory for creating indexer commands
     * @param feederCommandFactory        factory for creating feeder commands
     * @param intakeCommandFactory        factory for creating intake commands
     * @param harvesterCommandFactory     factory for creating harvester commands
     * @param gameplayStateCommandFactory factory for creating gameplay state transition commands
     */
    public CompetitionTriggerBindings(
            DriveBaseSubsystemCommandFactory driveBaseCommandFactory,
            TriggerBindingsConfig triggerBindingsConfig,
            TurretSubsystemCommandFactory turretCommandFactory,
            ShooterSubsystemCommandFactory shooterCommandFactory,
            IndexerSubsystemCommandFactory indexerCommandFactory,
            FeederSubsystemCommandFactory feederCommandFactory,
            IntakeSubsystemCommandFactory intakeCommandFactory,
            HarvesterSubsystemCommandFactory harvesterCommandFactory,
            GameplayStateCommandFactory gameplayStateCommandFactory) {
        super(
                driveBaseCommandFactory,
                triggerBindingsConfig,
                turretCommandFactory,
                shooterCommandFactory,
                indexerCommandFactory,
                feederCommandFactory,
                intakeCommandFactory,
                harvesterCommandFactory,
                gameplayStateCommandFactory);
    }

    /**
     * Wires the driver controller sticks and triggers to field-relative driving commands.
     * <p>
     * Left stick controls translation (Y = forward/back, X = strafe), right stick X controls rotation rate. Triggers select a speed tier (slow,
     * normal, sprint). Response curve exponents and speed scales are read from tunable config each cycle.
     * </p>
     */
    @Override
    protected void assignDriverControls() {
        configureManualDriveCommand();
        configureGameplayStateShortcuts();
        configureHeadingCommands();
        configureWheelLockCommand();
        configureDpadPathfindingBindings();
    }

    /**
     * Wires operator controller buttons to gameplay state transition commands.
     * <p>
     * Y enters FIRE_READY, X enters HARVEST_READY, A enters EJECT, back enters IDLE, right trigger enters TRAVEL, and left trigger enters
     * TRENCH_TRAVEL. D-pad up/down adjusts shooter RPM. B button toggles turret lock to zero degrees.
     * </p>
     */
    @Override
    protected void assignOperatorControls() {
        configureOperatorGameplayStateBindings();
        configureOperatorShooterAdjustments();
        configureOperatorTurretLock();
    }

    /**
     * Maps driver sticks to field-relative driving using the drive base command factory.
     */
    private void configureManualDriveCommand() {
        driveBaseCommandFactory.setDefaultManualDriveCommand(
                () -> applyResponseCurve(
                        driverController.getLeftY(),
                        triggerBindingsConfig.getLeftStickYResponseExponent(),
                        computeTranslationSpeedScale()),
                () -> applyResponseCurve(
                        driverController.getLeftX(),
                        triggerBindingsConfig.getLeftStickXResponseExponent(),
                        computeTranslationSpeedScale()),
                () -> applyResponseCurve(
                        driverController.getRightX(),
                        triggerBindingsConfig.getRightStickXResponseExponent(),
                        1.0));
    }

    /**
     * Maps driver bumpers to travel and trench-travel gameplay states.
     */
    private void configureGameplayStateShortcuts() {
        // Right bumper: travel mode (stow harvester and idle all mechanisms).
        debounce(driverController.rightBumper()).onTrue(
                gameplayStateCommandFactory.createTravelCommand());

        // Left bumper: trench travel mode (deploy harvester for low-profile field traversal).
        debounce(driverController.leftBumper()).onTrue(
                gameplayStateCommandFactory.createTrenchTravelCommand());
    }

    /**
     * Maps driver X and Y buttons to heading lock commands.
     */
    private void configureHeadingCommands() {
        // X button: spin 180 degrees from current heading. Hold to maintain heading lock;
        // release to return to normal manual rotation control.
        debounce(driverController.x()).whileTrue(
                driveBaseCommandFactory.createSpin180Command(
                        () -> applyResponseCurve(
                                driverController.getLeftY(),
                                triggerBindingsConfig.getLeftStickYResponseExponent(),
                                computeTranslationSpeedScale()),
                        () -> applyResponseCurve(
                                driverController.getLeftX(),
                                triggerBindingsConfig.getLeftStickXResponseExponent(),
                                computeTranslationSpeedScale())));

        // Y button: snap to nearest field-facing orientation (0 or 180 degrees field-relative).
        // If already at the nearest orientation, snaps to the opposite one instead.
        // Hold to maintain heading lock; release to return to normal manual rotation control.
        debounce(driverController.y()).whileTrue(
                driveBaseCommandFactory.createSnapToFieldFacingCommand(
                        () -> applyResponseCurve(
                                driverController.getLeftY(),
                                triggerBindingsConfig.getLeftStickYResponseExponent(),
                                computeTranslationSpeedScale()),
                        () -> applyResponseCurve(
                                driverController.getLeftX(),
                                triggerBindingsConfig.getLeftStickXResponseExponent(),
                                computeTranslationSpeedScale())));
    }

    /**
     * Maps driver back button to the wheel lock (X-formation) command.
     */
    private void configureWheelLockCommand() {
        // Back button: lock wheels in X formation as a defensive stance.
        // One-shot command — the lock fires once and normal driving resumes with stick input.
        debounce(driverController.back()).onTrue(
                driveBaseCommandFactory.createWheelLockCommand());
    }

    /**
     * Wires driver controller d-pad buttons to PathPlanner pathfinding commands.
     * <p>
     * Each d-pad direction pathfinds to a configurable field pose when held. Only directions with {@code enabled = true} in the config are bound.
     * Alliance flipping, trench zone detection, and constraint resolution are handled by the drive base command factory.
     * </p>
     */
    private void configureDpadPathfindingBindings() {
        DriverControllerConfig driverConfig = triggerBindingsConfig.driverControllerConfig;

        bindDpadDirection(debounce(driverController.povUp()), driverConfig.dpadUp, driverConfig);
        bindDpadDirection(debounce(driverController.povDown()), driverConfig.dpadDown, driverConfig);
        bindDpadDirection(debounce(driverController.povLeft()), driverConfig.dpadLeft, driverConfig);
        bindDpadDirection(debounce(driverController.povRight()), driverConfig.dpadRight, driverConfig);
    }

    /**
     * Binds a single d-pad direction trigger to a pathfinding command if the target is enabled.
     *
     * @param trigger      the d-pad direction trigger from the driver controller
     * @param targetConfig config holding the target pose for this direction
     * @param driverConfig shared driver controller config holding pathfinding constraints and trench zone definitions
     */
    private void bindDpadDirection(
            edu.wpi.first.wpilibj2.command.button.Trigger trigger,
            DpadTargetConfig targetConfig,
            DriverControllerConfig driverConfig) {
        if (!targetConfig.enabled) {
            return;
        }

        trigger.whileTrue(driveBaseCommandFactory.createDpadPathfindCommand(
                targetConfig, driverConfig));
    }

    /**
     * Maps operator buttons to gameplay state transitions (fire, harvest, eject, idle, travel, trench travel).
     */
    private void configureOperatorGameplayStateBindings() {
        // Y button: fire fuel.
        debounce(operatorController.y()).whileTrue(
                gameplayStateCommandFactory.createFireReadyCommand());

        // X button: harvest fuel.
        debounce(operatorController.x()).onTrue(
                gameplayStateCommandFactory.createHarvestReadyCommand());

        // A button: eject all fuel.
        debounce(operatorController.a()).whileTrue(
                gameplayStateCommandFactory.createEjectCommand());

        // Start button: open fire (diagnostic, bypasses readiness checks).
        debounce(operatorController.start()).whileTrue(
                gameplayStateCommandFactory.createOpenFireCommand());

        // Back button: return to idle.
        debounce(operatorController.back()).onTrue(
                gameplayStateCommandFactory.createIdleCommand());

        // Right trigger: travel mode (stow harvester and idle all mechanisms).
        operatorController.rightTrigger().onTrue(
                gameplayStateCommandFactory.createTravelCommand());

        // Left trigger: trench travel mode (deploy harvester for low-profile field traversal).
        operatorController.leftTrigger().onTrue(
                gameplayStateCommandFactory.createTrenchTravelCommand());
    }

    /**
     * Maps operator d-pad up/down to shooter RPM boost and cut commands.
     */
    private void configureOperatorShooterAdjustments() {
        // D-pad up: boost shooter RPM by the configured adjustment amount while held.
        debounce(operatorController.povUp()).whileTrue(
                shooterCommandFactory.createBoostRpmCommand());

        // D-pad down: cut shooter RPM by the configured adjustment amount while held.
        debounce(operatorController.povDown()).whileTrue(
                shooterCommandFactory.createCutRpmCommand());
    }

    /**
     * Maps operator B button to a toggle that locks the turret at zero degrees.
     */
    private void configureOperatorTurretLock() {
        // B button: toggle turret lock to 0 degrees.
        // First press locks the turret at 0 and disables field tracking.
        // Second press releases the lock and resumes the default tracking command.
        debounce(operatorController.b()).toggleOnTrue(
                turretCommandFactory.createLockToZeroCommand());
    }
}
