package frc.robot.shared.bindings;

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

    private static final int                       DEFAULT_DRIVE_CONTROLLER_PORT = 0;

    private final GameController                   driverController;

    private final DriveBaseSubsystemCommandFactory driveBaseCommandFactory;

    private final TurretSubsystemCommandFactory    turretCommandFactory;

    /**
     * Creates trigger bindings with the default drive controller port (0).
     *
     * @param driveBaseCommandFactory factory for creating drive base commands
     * @param driveBaseConfig         configuration used to scale joystick inputs into real speeds
     * @param turretCommandFactory    factory for creating turret commands
     */
    public TriggerBindings(
            DriveBaseSubsystemCommandFactory driveBaseCommandFactory,
            DriveBaseSubsystemConfig driveBaseConfig,
            TurretSubsystemCommandFactory turretCommandFactory) {
        this(driveBaseCommandFactory, driveBaseConfig, turretCommandFactory, DEFAULT_DRIVE_CONTROLLER_PORT);
    }

    /**
     * Creates trigger bindings using an explicit drive controller port.
     *
     * @param driveBaseCommandFactory factory for creating drive base commands
     * @param driveBaseConfig         configuration used to scale joystick inputs into real speeds
     * @param driveControllerPort     USB port for the driver controller
     * @param turretCommandFactory    factory for creating turret commands
     */
    public TriggerBindings(
            DriveBaseSubsystemCommandFactory driveBaseCommandFactory,
            DriveBaseSubsystemConfig driveBaseConfig,
            TurretSubsystemCommandFactory turretCommandFactory,
            int driveControllerPort) {
        this.driveBaseCommandFactory = driveBaseCommandFactory;
        this.turretCommandFactory    = turretCommandFactory;
        this.driverController        = new GameController(driveControllerPort);

        configureDriveControllerBindings();
        configureTurretBindings();
    }

    private void configureDriveControllerBindings() {
        driveBaseCommandFactory.setDefaultManualDriveCommand(
                () -> driverController.getRawAxis(GameControllerAxes.LeftStickY.getValue()),
                () -> driverController.getRawAxis(GameControllerAxes.LeftStickX.getValue()),
                () -> driverController.getRawAxis(GameControllerAxes.RightStickX.getValue()));
    }

    private void configureTurretBindings() {
        driverController.onButtonHold(
                GameControllerButton.A,
                turretCommandFactory.createMoveToAngleCommand(-360.0));
        driverController.onButtonHold(
                GameControllerButton.B,
                turretCommandFactory.createMoveToAngleCommand(360.0));
    }

}
