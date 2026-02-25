package frc.robot.subsystems.harvester;

import frc.robot.devices.motor.Motor;
import frc.robot.shared.config.RobotEnvironment;
import frc.robot.shared.subsystems.AbstractSetAndSeekSubsystem;
import frc.robot.subsystems.harvester.config.HarvesterSubsystemConfig;
import frc.robot.subsystems.harvester.devices.HarvesterMotor;
import frc.robot.subsystems.harvester.devices.HarvesterSimMotor;

/**
 * Harvester subsystem that swings the intake arm between a stowed (upright) position and a deployed (lowered) position using a profiled motor. The
 * superclass handles motion profiling, limits, and logging while this class exposes convenience methods for the two named positions.
 * <p>
 * A trapezoidal motion profile is a control technique that limits both the velocity and acceleration of the motor, producing smooth, predictable
 * movements instead of abrupt starts and stops.
 * </p>
 */
public class HarvesterSubsystem extends AbstractSetAndSeekSubsystem<HarvesterSubsystemConfig> {

    /**
     * Builds the hardware or simulation motor wrapper based on the current robot environment.
     *
     * @param config harvester configuration bundle loaded from JSON
     * @return configured motor wrapper, or null when the subsystem is disabled (parent substitutes a {@code DisabledMotor})
     */
    private static Motor buildMotor(HarvesterSubsystemConfig config) {
        if (!config.enabled) {
            return null;
        }

        return RobotEnvironment.isReal()
                ? HarvesterMotor.create(config.harvesterMotorConfig)
                : HarvesterSimMotor.create(
                        config.harvesterMotorConfig,
                        config::getMaximumVelocityDegreesPerSecond,
                        config::getMaximumAccelerationDegreesPerSecondSquared);
    }

    /**
     * Builds the harvester subsystem with a single profiled motor for arm positioning.
     *
     * @param config harvester configuration bundle loaded from JSON; angles are expressed in degrees
     */
    public HarvesterSubsystem(HarvesterSubsystemConfig config) {
        this(config, buildMotor(config));
    }

    private HarvesterSubsystem(HarvesterSubsystemConfig config, Motor motor) {
        super(config, motor);
    }

    /**
     * Commands the arm to deploy to the lowered Fuel-collection position.
     * <p>
     * The motion profile ramps the arm smoothly from its current position to the configured deployed angle. The arm will stay at the target until a
     * new target is set or the command is interrupted.
     * </p>
     */
    public void deployArm() {
        if (isSubsystemDisabled()) {
            logDisabled("deployArm");
            return;
        }
        setTarget(config.getDeployedPositionDegrees());
    }

    /**
     * Commands the arm to stow in the upright match-start position.
     * <p>
     * The motion profile ramps the arm smoothly from its current position to the configured stowed angle. This is the default position at the start
     * of a match and keeps the intake inside the robot perimeter.
     * </p>
     */
    public void stowArm() {
        if (isSubsystemDisabled()) {
            logDisabled("stowArm");
            return;
        }
        setTarget(config.getStowedPositionDegrees());
    }
}
