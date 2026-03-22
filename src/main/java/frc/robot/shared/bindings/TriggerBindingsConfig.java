package frc.robot.shared.bindings;

import frc.robot.shared.config.AbstractConfig;

/**
 * Configuration bundle for driver controller input shaping and speed tiers. Each stick axis has an independent response exponent that shapes the
 * joystick curve, and three speed scales control how fast the robot drives depending on which trigger the driver is holding.
 * <p>
 * The speed scales are absolute fractions of the robot's configured maximum speed. For example, a normal speed scale of 0.8 means the driver can
 * reach 80 percent of the robot's theoretical top speed with the stick at full deflection and no trigger held. This keeps all speed tuning in one
 * place rather than splitting it between the controller config and the drive base config.
 * </p>
 */
public class TriggerBindingsConfig extends AbstractConfig {

    /**
     * Response curve exponent for the left stick Y axis (forward/backward translation).
     * <p>
     * Applied before the speed scale so it sets the baseline feel. A value of 2.0 (quadratic) is a good starting point for most drivers: small stick
     * movements give fine control, while full deflection still reaches max speed.
     * </p>
     */
    public double  leftStickYResponseExponent  = 2.0;

    /**
     * Response curve exponent for the left stick X axis (left/right translation).
     * <p>
     * Applied before the speed scale so it sets the baseline feel. Typically matches the Y axis exponent for consistent translation response.
     * </p>
     */
    public double  leftStickXResponseExponent  = 2.0;

    /**
     * Response curve exponent for the right stick X axis (rotation/omega).
     * <p>
     * Applied independently of the speed scale since rotation is not affected by triggers. A value of 2.0 makes small rotational corrections easier
     * without reducing max turn rate.
     * </p>
     */
    public double  rightStickXResponseExponent = 2.0;

    /**
     * Fraction of maximum linear speed used during normal driving (no trigger held).
     * <p>
     * A value of 0.8 means fully deflecting the stick reaches 80 percent of the robot's configured top speed. This is the default "gear" the driver
     * lives in most of the time.
     * </p>
     */
    public double  normalSpeedScale            = 0.8;

    /**
     * Fraction of maximum linear speed used when the right trigger (sprint) is held.
     * <p>
     * A value of 1.0 means fully deflecting the stick reaches 100 percent of the robot's configured top speed. Think of it like a sprint button in a
     * video game.
     * </p>
     */
    public double  sprintSpeedScale            = 1.0;

    /**
     * Fraction of maximum linear speed used when the left trigger (precision/slow) is held.
     * <p>
     * A value of 0.4 means fully deflecting the stick reaches 40 percent of the robot's configured top speed. Useful for precise alignment and
     * slow-speed maneuvering.
     * </p>
     */
    public double  slowSpeedScale              = 0.4;

    /**
     * Minimum trigger axis value required to consider a trigger "pressed."
     * <p>
     * Values below this threshold are ignored to prevent accidental activation from controller noise or resting position drift.
     * </p>
     */
    public double  triggerDeadband             = 0.1;

    /**
     * Joystick deadband for driver stick inputs.
     * <p>
     * Stick values inside this range are treated as zero to ignore noise when the stick is at rest. A typical value is 0.08 (8 percent of full
     * deflection). Applied before the response curve so that stick noise is eliminated before the power function can amplify it.
     * </p>
     */
    public double  joystickDeadband            = 0.08;

    /**
     * Enables tuning mode for trigger bindings.
     * <p>
     * When {@code true}, the driver controller A/B/X buttons are mapped to subsystem test commands (min/max setpoints and SysId sweeps) via a
     * dashboard-selectable subsystem chooser, and default commands are not registered so mechanisms stay where test commands leave them. When
     * {@code false} (the default), production gameplay bindings and operator state-transition buttons are active instead.
     * </p>
     * <p>
     * Because WPILib trigger bindings are wired once at construction, a code restart is required after changing this value.
     * </p>
     */
    public boolean tuningEnabled               = false;

    /**
     * Driver controller configuration for d-pad pathfinding targets and constraints.
     */
    public DriverControllerConfig driverControllerConfig = new DriverControllerConfig();

    /**
     * Reads the tunable response curve exponent for the left stick Y axis.
     *
     * @return current left stick Y response exponent (1.0 = linear, 2.0 = quadratic)
     */
    public double getLeftStickYResponseExponent() {
        return readTunableNumber("leftStickYResponseExponent", leftStickYResponseExponent);
    }

    /**
     * Reads the tunable response curve exponent for the left stick X axis.
     *
     * @return current left stick X response exponent (1.0 = linear, 2.0 = quadratic)
     */
    public double getLeftStickXResponseExponent() {
        return readTunableNumber("leftStickXResponseExponent", leftStickXResponseExponent);
    }

    /**
     * Reads the tunable response curve exponent for the right stick X axis.
     *
     * @return current right stick X response exponent (1.0 = linear, 2.0 = quadratic)
     */
    public double getRightStickXResponseExponent() {
        return readTunableNumber("rightStickXResponseExponent", rightStickXResponseExponent);
    }

    /**
     * Reads the tunable normal (no trigger) speed scale.
     *
     * @return fraction of max linear speed during normal driving (e.g., 0.8 = 80 percent)
     */
    public double getNormalSpeedScale() {
        return readTunableNumber("normalSpeedScale", normalSpeedScale);
    }

    /**
     * Reads the tunable sprint (right trigger) speed scale.
     *
     * @return fraction of max linear speed during sprint (e.g., 1.0 = 100 percent)
     */
    public double getSprintSpeedScale() {
        return readTunableNumber("sprintSpeedScale", sprintSpeedScale);
    }

    /**
     * Reads the tunable slow (left trigger) speed scale.
     *
     * @return fraction of max linear speed during slow/precision mode (e.g., 0.4 = 40 percent)
     */
    public double getSlowSpeedScale() {
        return readTunableNumber("slowSpeedScale", slowSpeedScale);
    }

    /**
     * Reads the tunable deadband threshold for trigger activation.
     *
     * @return minimum trigger axis value to count as pressed
     */
    public double getTriggerDeadband() {
        return readTunableNumber("triggerDeadband", triggerDeadband);
    }

    /**
     * Reads the tunable joystick deadband for driver stick inputs.
     *
     * @return current joystick deadband (fraction of full deflection, e.g., 0.08)
     */
    public double getJoystickDeadband() {
        return readTunableNumber("joystickDeadband", joystickDeadband);
    }

    /**
     * Reads whether tuning mode is enabled for trigger bindings.
     * <p>
     * Because bindings are wired at construction, this value is only meaningful at startup. The SmartDashboard entry is still published so operators
     * can see which mode is active.
     * </p>
     *
     * @return {@code true} if tuning mode is active, {@code false} for gameplay mode
     */
    public boolean getTuningEnabled() {
        return readTunableBoolean("tuningEnabled", tuningEnabled);
    }
}
