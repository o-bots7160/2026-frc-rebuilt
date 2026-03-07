package frc.robot.subsystems.harvester;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.devices.motor.Motor;
import frc.robot.shared.config.RobotEnvironment;
import frc.robot.shared.subsystems.AbstractSetAndSeekSubsystem;
import frc.robot.subsystems.harvester.config.HarvesterSubsystemConfig;
import frc.robot.subsystems.harvester.devices.HarvesterMotor;
import frc.robot.subsystems.harvester.devices.HarvesterSimMotor;

/**
 * Harvester subsystem that swings the intake arm between a stowed (upright) position and a deployed (lowered) position using a profiled motor. The
 * superclass handles motion profiling, limits, and logging while this class adds gravity-compensating arm feedforward and exposes convenience methods
 * for the two named positions.
 * <p>
 * A trapezoidal motion profile is a control technique that limits both the velocity and acceleration of the motor, producing smooth, predictable
 * movements instead of abrupt starts and stops.
 * </p>
 * <p>
 * Because the arm swings against gravity, this subsystem replaces the parent's simple feedforward with an
 * {@link ArmFeedforward}. The arm feedforward adds a gravity term (kG) that is multiplied by the cosine of the
 * arm's angle from horizontal, automatically varying the compensation voltage as the arm moves.
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
     * Arm feedforward model that accounts for gravity by using the arm's angular position.
     */
    private ArmFeedforward armFeedforward;

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

        // Build the arm feedforward with gravity compensation.
        armFeedforward = new ArmFeedforward(
                config.getkS(),
                config.getkG(),
                config.getkV(),
                config.getkA());
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

    /**
     * Computes the arm feedforward voltage including gravity compensation for the current profile step.
     * <p>
     * The arm angle from horizontal is computed by adding the configured horizontal offset to the current setpoint position. The gravity term
     * (kG × cos(angle)) varies automatically as the arm moves, producing more voltage when the arm is near horizontal and less when it is vertical.
     * </p>
     *
     * @param previousSetpointVelocity velocity setpoint from the previous cycle in radians per second
     * @param currentSetpoint          the setpoint state the profile wants us to follow this cycle
     * @return feedforward voltage in volts, including gravity compensation
     */
    @Override
    protected double calculateFeedforward(double previousSetpointVelocity, TrapezoidProfile.State currentSetpoint) {
        // The horizontal offset maps encoder zero to the true angle from horizontal.
        double armAngleFromHorizontalRadians = currentSetpoint.position + config.getHorizontalOffsetRadians();

        log.recordVerboseOutput("armAngleFromHorizontalDegrees",
                Math.toDegrees(armAngleFromHorizontalRadians));

        return armFeedforward.calculateWithVelocities(
                armAngleFromHorizontalRadians,
                previousSetpointVelocity,
                currentSetpoint.velocity);
    }

    /**
     * Re-reads arm feedforward gains from config so live tuning updates affect voltage estimates immediately.
     */
    @Override
    protected void refreshFeedforward() {
        super.refreshFeedforward();
        armFeedforward = new ArmFeedforward(
                config.getkS(),
                config.getkG(),
                config.getkV(),
                config.getkA());
    }
}
