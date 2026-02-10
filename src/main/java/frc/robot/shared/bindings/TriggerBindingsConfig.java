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
     * Base value for the right trigger (speed-up) throttle scale.
     * <p>
     * The raw trigger axis is subtracted from this value, so a higher base means a faster default
     * speed before the trigger is pressed. At 1.0 the robot starts at full speed and slows as the
     * trigger is released.
     * </p>
     */
    public double rightTriggerBaseScale = 1.0;

    /**
     * Base value for the left trigger (slow-down) throttle scale.
     * <p>
     * Works the same way as {@code rightTriggerBaseScale} but for the braking trigger.
     * </p>
     */
    public double leftTriggerBaseScale  = 1.0;

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
     * Reads the tunable base scale for the right trigger (speed-up).
     *
     * @return current right trigger base scale
     */
    public double getRightTriggerBaseScale() {
        return readTunableNumber("rightTriggerBaseScale", rightTriggerBaseScale);
    }

    /**
     * Reads the tunable base scale for the left trigger (slow-down).
     *
     * @return current left trigger base scale
     */
    public double getLeftTriggerBaseScale() {
        return readTunableNumber("leftTriggerBaseScale", leftTriggerBaseScale);
    }
}
