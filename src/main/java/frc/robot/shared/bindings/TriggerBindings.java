package frc.robot.shared.bindings;

import edu.wpi.first.math.MathUtil;
import frc.robot.devices.GameController;
import frc.robot.devices.GameController.GameControllerAxes;
import frc.robot.devices.GameController.GameControllerButton;
import frc.robot.subsystems.drivebase.commands.DriveBaseSubsystemCommandFactory;
import frc.robot.subsystems.drivebase.config.DriveBaseSubsystemConfig;
import frc.robot.subsystems.turret.commands.TurretSubsystemCommandFactory;

/**
 * Maps the driver controller to robot commands so RobotContainer stays lean. Currently wires the drive controller to the default manual drive
 * command.
 */
public class TriggerBindings {

    /**
     * Default USB port for the driver controller.
     */
    private static final int                       DEFAULT_DRIVE_CONTROLLER_PORT    = 0;

    /**
     * Default USB port for the operator controller.
     */
    private static final int                       DEFAULT_OPERATOR_CONTROLLER_PORT = 1;

    /**
     * Driver gamepad used for manual driving.
     */
    private final GameController                   driverController;

    /**
     * Operator gamepad used for mechanism control (ex: turret).
     */
    private final GameController                   operatorController;

    /**
     * Factory that creates drive base commands tied to the driver inputs.
     */
    private final DriveBaseSubsystemCommandFactory driveBaseCommandFactory;

    /**
     * Factory that creates turret commands tied to driver buttons.
     */
    private final TurretSubsystemCommandFactory    turretCommandFactory;

    /**
     * Creates trigger bindings with the default driver and operator controller ports.
     *
     * @param driveBaseCommandFactory factory for creating drive base commands
     * @param driveBaseConfig         configuration used to scale joystick inputs into real speeds
     * @param turretCommandFactory    factory for creating turret commands
     */
    public TriggerBindings(
            DriveBaseSubsystemCommandFactory driveBaseCommandFactory,
            DriveBaseSubsystemConfig driveBaseConfig,
            TurretSubsystemCommandFactory turretCommandFactory) {
        this(
                driveBaseCommandFactory,
                driveBaseConfig,
                turretCommandFactory,
                DEFAULT_DRIVE_CONTROLLER_PORT,
                DEFAULT_OPERATOR_CONTROLLER_PORT);
    }

    /**
     * Creates trigger bindings using explicit controller ports.
     *
     * @param driveBaseCommandFactory factory for creating drive base commands
     * @param driveBaseConfig         configuration used to scale joystick inputs into real speeds
     * @param driveControllerPort     USB port for the driver controller
     * @param operatorControllerPort  USB port for the operator controller
     * @param turretCommandFactory    factory for creating turret commands
     */
    public TriggerBindings(
            DriveBaseSubsystemCommandFactory driveBaseCommandFactory,
            DriveBaseSubsystemConfig driveBaseConfig,
            TurretSubsystemCommandFactory turretCommandFactory,
            int driveControllerPort,
            int operatorControllerPort) {
        this.driveBaseCommandFactory = driveBaseCommandFactory;
        this.turretCommandFactory    = turretCommandFactory;
        this.driverController        = new GameController(driveControllerPort);
        this.operatorController      = new GameController(operatorControllerPort);

        configureDriveControllerBindings();
        configureTurretBindings();
    }

    private void configureDriveControllerBindings() {
        // Map sticks to field-relative driving using the drive base command factory.
        // Left stick: translation (forward/back on Y, left/right on X).
        // Right stick X: rotation rate (omega).
        // Triggers: scale translation speed for slow/fast control.
        driveBaseCommandFactory.setDefaultManualDriveCommand(
                () -> MathUtil.clamp(
                        driverController.getRawAxis(GameControllerAxes.LeftStickY.getValue())
                                * computeDriveThrottleScale(),
                        -1.0,
                        1.0),
                () -> MathUtil.clamp(
                        driverController.getRawAxis(GameControllerAxes.LeftStickX.getValue())
                                * computeDriveThrottleScale(),
                        -1.0,
                        1.0),
                () -> driverController.getRawAxis(GameControllerAxes.RightStickX.getValue()));
    }

    private double computeDriveThrottleScale() {
        // Right trigger increases speed, left trigger reduces speed.
        // Add a small offset so we never divide by zero when fully pressed.
        double speedUp  = 1.0 - driverController.getRawAxis(GameControllerAxes.RTrigger.getValue());
        double slowDown = 1.0 - driverController.getRawAxis(GameControllerAxes.LTrigger.getValue());
        return (speedUp + 0.001) / (slowDown + 0.001);
    }

    private void configureTurretBindings() {
        // Hold A/B to spin the turret to the configured angles (for testing and alignment).
        driverController.onButtonHold(
                GameControllerButton.A,
                turretCommandFactory.createMoveToAngleCommand(-270.0));
        driverController.onButtonHold(
                GameControllerButton.B,
                turretCommandFactory.createMoveToAngleCommand(270.0));
    }

}
