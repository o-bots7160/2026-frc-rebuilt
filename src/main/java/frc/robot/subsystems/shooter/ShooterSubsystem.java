package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import frc.robot.devices.motor.Motor;
import frc.robot.shared.RobotEnvironment;
import frc.robot.shared.subsystems.AbstractVelocitySubsystem;
import frc.robot.subsystems.shooter.config.ShooterSubsystemConfig;
import frc.robot.subsystems.shooter.devices.ShooterMotor;
import frc.robot.subsystems.shooter.devices.ShooterSimMotor;

/**
 * Shooter subsystem that spins a flywheel to launch Fuel into the scoring hub.
 * <p>
 * All RPM values in the public API represent flywheel (mechanism) speed after gear reduction, not motor shaft speed. The gear ratio is applied at the
 * motor encoder conversion layer, so callers never deal with motor-side rotations.
 * </p>
 * <p>
 * The subsystem uses a feedforward model to estimate the voltage needed for a target velocity and a PID controller to correct for disturbances (such
 * as when a piece enters the shooter and momentarily slows the flywheel). An optional trapezoidal velocity ramp provides smooth spin-up when
 * configured.
 * </p>
 */
public class ShooterSubsystem extends AbstractVelocitySubsystem<ShooterSubsystemConfig> {

    /**
     * Builds the correct motor implementation for the current environment.
     * <p>
     * Returns null when the subsystem is disabled so the parent falls back to {@link frc.robot.devices.motor.DisabledMotor}.
     * </p>
     */
    private static Motor buildMotor(ShooterSubsystemConfig config) {
        if (!config.enabled) {
            return null;
        }

        // AbstractSimMotor expects degrees/sec suppliers. Convert from RPM: RPM × 6 = deg/s.
        return RobotEnvironment.isReal()
                ? ShooterMotor.create(config.shooterMotorConfig)
                : ShooterSimMotor.create(
                        config.shooterMotorConfig,
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
     * Builds the shooter subsystem with a single SparkMax-driven flywheel motor.
     *
     * @param config shooter configuration bundle loaded from JSON; velocities are expressed in RPM
     */
    public ShooterSubsystem(ShooterSubsystemConfig config) {
        this(config, buildMotor(config));
    }

    private ShooterSubsystem(ShooterSubsystemConfig config, Motor motor) {
        super(config, motor);
    }

    /**
     * Publishes shooter telemetry each cycle.
     */
    @Override
    public void periodic() {
        super.periodic();

        log.recordOutput("readyToFire", isReadyToFire());
        log.recordOutput("measuredRpm", getMeasuredVelocityRpm());
        log.recordOutput("targetRpm", getTargetVelocityRpm());
    }

    /**
     * Reports whether the flywheel is spinning fast enough and has been stable long enough to fire.
     * <p>
     * Other subsystems (such as the indexer) and autonomous commands should wait for this signal before feeding a piece into the shooter. The check
     * considers both velocity tolerance and the configured settle time to avoid firing during transient speed spikes.
     * </p>
     *
     * @return true when the flywheel is at the target RPM and ready to launch
     */
    public boolean isReadyToFire() {
        return isAtTargetVelocity();
    }
}
