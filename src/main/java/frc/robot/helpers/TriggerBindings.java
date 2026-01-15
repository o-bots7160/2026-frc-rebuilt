package frc.robot.helpers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.factories.DriveBaseSubsystemCommandFactory;
import frc.robot.config.DriveBaseSubsystemConfig;
import frc.robot.devices.GameController;
import frc.robot.devices.GameController.GameControllerAxes;

/**
 * Maps the driver controller to robot commands so RobotContainer stays lean. Currently wires the drive controller to the default manual drive
 * command.
 */
public class TriggerBindings {

    private static final int                       DEFAULT_DRIVE_CONTROLLER_PORT = 0;

    private final GameController                   driverController;

    private final DriveBaseSubsystemCommandFactory driveBaseCommandFactory;

    /**
    * Creates trigger bindings with the default drive controller port (0).
     *
     * @param driveBaseCommandFactory factory for creating drive base commands
     * @param driveBaseConfig         configuration used to scale joystick inputs into real speeds
     */
    public TriggerBindings(
            DriveBaseSubsystemCommandFactory driveBaseCommandFactory,
            DriveBaseSubsystemConfig driveBaseConfig) {
        this(driveBaseCommandFactory, driveBaseConfig, DEFAULT_DRIVE_CONTROLLER_PORT);
    }

    /**
     * Creates trigger bindings using an explicit drive controller port.
     *
     * @param driveBaseCommandFactory factory for creating drive base commands
     * @param driveBaseConfig         configuration used to scale joystick inputs into real speeds
     * @param driveControllerPort     USB port for the driver controller
     */
    public TriggerBindings(
            DriveBaseSubsystemCommandFactory driveBaseCommandFactory,
            DriveBaseSubsystemConfig driveBaseConfig,
            int driveControllerPort) {
        this.driveBaseCommandFactory = driveBaseCommandFactory;
        this.driverController        = new GameController(driveControllerPort);

        driveBaseCommandFactory.getSubsystem().setTranslationScaleSupplier(
        () -> SmartDashboard.getNumber("DriveBaseSubsystem/translationScale", driveBaseConfig.getTranslationScale().get()));
        driveBaseCommandFactory.getSubsystem().setAngularSpeedScaleSupplier(
                () -> SmartDashboard.getNumber("DriveBaseSubsystem/omegaScale", 1.0));

        configureDriveControllerBindings();
    }

    private void configureDriveControllerBindings() {
        driveBaseCommandFactory.setDefaultManualDriveCommand(
                () -> driverController.getRawAxis(GameControllerAxes.LeftStickY.getValue()),
                () -> driverController.getRawAxis(GameControllerAxes.LeftStickX.getValue()),
                () -> driverController.getRawAxis(GameControllerAxes.RightStickX.getValue()));
    }

}
