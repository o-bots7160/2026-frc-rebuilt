package frc.robot.subsystems.hopper;

import edu.wpi.first.math.util.Units;
import frc.robot.devices.motor.Motor;
import frc.robot.shared.config.RobotEnvironment;
import frc.robot.shared.subsystems.AbstractVelocitySubsystem;
import frc.robot.subsystems.hopper.config.HopperSubsystemConfig;
import frc.robot.subsystems.hopper.devices.HopperMotor;
import frc.robot.subsystems.hopper.devices.HopperSimMotor;

/**
 * Hopper subsystem that drives a belt to transport Fuel from the intake toward the feeder.
 * <p>
 * The hopper sits between the intake and the feeder. Its belt moves forward (positive RPM) to carry Fuel inward toward the feeder, or reverses
 * (negative RPM) to purge Fuel back through the intake when clearing jams or ejecting unwanted pieces.
 * </p>
 * <p>
 * All RPM values in the public API represent belt (mechanism) speed after gear reduction, not motor shaft speed. The subsystem extends
 * {@link AbstractVelocitySubsystem} which provides bidirectional velocity control with feedforward and PID.
 * </p>
 */
public class HopperSubsystem extends AbstractVelocitySubsystem<HopperSubsystemConfig> {

    /**
     * Builds the correct motor implementation for the current environment.
     * <p>
     * Returns null when the subsystem is disabled so the parent falls back to {@link frc.robot.devices.motor.DisabledMotor}.
     * </p>
     */
    private static Motor buildMotor(HopperSubsystemConfig config) {
        if (!config.enabled) {
            return null;
        }

        return RobotEnvironment.isReal()
                ? HopperMotor.create(config.hopperMotorConfig)
                : HopperSimMotor.create(
                        config.hopperMotorConfig,
                        () -> rpmToDegreesPerSecond(config.getMaximumVelocityRpm()),
                        () -> rpmToDegreesPerSecond(config.getMaximumAccelerationRpmPerSecond()));
    }

    /**
     * Converts RPM to degrees per second.
     * <p>
     * Used internally to convert from the RPM-based config to the degrees-per-second suppliers expected by the simulation motor.
     * </p>
     *
     * @param rpm value in rotations per minute
     * @return equivalent value in degrees per second
     */
    private static double rpmToDegreesPerSecond(double rpm) {
        return Units.radiansToDegrees(Units.rotationsPerMinuteToRadiansPerSecond(rpm));
    }

    /**
     * Builds the hopper subsystem with a single motor-driven belt.
     *
     * @param config hopper configuration bundle loaded from JSON; velocities are expressed in RPM
     */
    public HopperSubsystem(HopperSubsystemConfig config) {
        this(config, buildMotor(config));
    }

    private HopperSubsystem(HopperSubsystemConfig config, Motor motor) {
        super(config, motor);
    }

    /**
     * Publishes hopper telemetry each cycle.
     */
    @Override
    public void periodic() {
        super.periodic();

        log.recordOutput("isTransporting", isTransporting());
        log.recordOutput("measuredRpm", getMeasuredVelocityRpm());
        log.recordOutput("targetRpm", getTargetVelocityRpm());
    }

    /**
     * Returns the hopper configuration bundle for command factories that need access to hopper-specific settings.
     *
     * @return hopper subsystem configuration
     */
    public HopperSubsystemConfig getConfig() {
        return config;
    }

    /**
     * Reports whether the belt is spinning fast enough and has been stable long enough to reliably transport Fuel.
     * <p>
     * Composite commands can wait for this signal before advancing to the next step in a fuel staging sequence.
     * </p>
     *
     * @return true when the belt is at the target RPM and has been stable for the configured settle time
     */
    public boolean isTransporting() {
        return isAtTargetVelocity();
    }

    /**
     * Convenience method that sets the belt to the configured forward transport velocity.
     * <p>
     * Use this to move Fuel from the intake side toward the feeder.
     * </p>
     */
    public void setForwardVelocity() {
        if (isSubsystemDisabled()) {
            logDisabled("setForwardVelocity");
            return;
        }
        setTargetVelocityRpm(config.getForwardVelocityRpm());
    }

    /**
     * Convenience method that sets the belt to reverse at the configured purge velocity.
     * <p>
     * The configured reverse RPM is stored as a positive value; this method negates it so the belt spins backward toward the intake.
     * </p>
     */
    public void setReverseVelocity() {
        if (isSubsystemDisabled()) {
            logDisabled("setReverseVelocity");
            return;
        }
        setTargetVelocityRpm(-config.getReverseVelocityRpm());
    }
}
