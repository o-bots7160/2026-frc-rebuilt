package frc.robot.shared.bindings;

import frc.robot.shared.config.AbstractConfig;

/**
 * Configuration bundle for driver and operator controller sensitivity. Each axis has an independent sensitivity multiplier that scales the raw
 * joystick value before it reaches the drive command.
 * <p>
 * A value of 1.0 means full sensitivity (no change). Values below 1.0 reduce sensitivity, making the robot feel slower and easier to control. Values
 * above 1.0 amplify stick response.
 * </p>
 */
public class TriggerBindingsConfig extends AbstractConfig {

    /**
     * Sensitivity multiplier for the left stick Y axis (forward/backward translation).
     * <p>
     * Applied before the trigger-based throttle scale so it sets the baseline responsiveness.
     * </p>
     */
    public double leftStickYSensitivity  = 1.0;

    /**
     * Sensitivity multiplier for the left stick X axis (left/right translation).
     * <p>
     * Applied before the trigger-based throttle scale so it sets the baseline responsiveness.
     * </p>
     */
    public double leftStickXSensitivity  = 1.0;

    /**
     * Sensitivity multiplier for the right stick X axis (rotation/omega).
     * <p>
     * Applied independently of the trigger throttle since rotation is not scaled by triggers.
     * </p>
     */
    public double rightStickXSensitivity = 1.0;

    /**
     * Speed multiplier applied when the right trigger (speed-up) is pressed.
     * <p>
     * A value of 1.5 means the drive input is scaled to 150 percent of its normal value. The factor is applied as a flat multiplier regardless of how
     * far the trigger is pressed. If both triggers are held, the speed-up trigger takes priority.
     * </p>
     */
    public double speedUpTriggerFactor   = 1.5;

    /**
     * Speed multiplier applied when the left trigger (slow-down) is pressed.
     * <p>
     * A value of 0.5 means the drive input is scaled to 50 percent of its normal value. The factor is applied as a flat multiplier regardless of how
     * far the trigger is pressed.
     * </p>
     */
    public double slowDownTriggerFactor  = 0.5;

    /**
     * Minimum trigger axis value required to consider a trigger "pressed."
     * <p>
     * Values below this threshold are ignored to prevent accidental activation from controller noise or resting position drift.
     * </p>
     */
    public double triggerDeadband        = 0.1;

    /**
     * Reads the tunable sensitivity for the left stick Y axis.
     *
     * @return current left stick Y sensitivity multiplier
     */
    public double getLeftStickYSensitivity() {
        return readTunableNumber("leftStickYSensitivity", leftStickYSensitivity);
    }

    /**
     * Reads the tunable sensitivity for the left stick X axis.
     *
     * @return current left stick X sensitivity multiplier
     */
    public double getLeftStickXSensitivity() {
        return readTunableNumber("leftStickXSensitivity", leftStickXSensitivity);
    }

    /**
     * Reads the tunable sensitivity for the right stick X axis.
     *
     * @return current right stick X sensitivity multiplier
     */
    public double getRightStickXSensitivity() {
        return readTunableNumber("rightStickXSensitivity", rightStickXSensitivity);
    }

    /**
     * Reads the tunable speed-up factor for the right trigger.
     *
     * @return current speed-up trigger factor
     */
    public double getSpeedUpTriggerFactor() {
        return readTunableNumber("speedUpTriggerFactor", speedUpTriggerFactor);
    }

    /**
     * Reads the tunable slow-down factor for the left trigger.
     *
     * @return current slow-down trigger factor
     */
    public double getSlowDownTriggerFactor() {
        return readTunableNumber("slowDownTriggerFactor", slowDownTriggerFactor);
    }

    /**
     * Reads the tunable deadband threshold for trigger activation.
     *
     * @return minimum trigger axis value to count as pressed
     */
    public double getTriggerDeadband() {
        return readTunableNumber("triggerDeadband", triggerDeadband);
    }
}
