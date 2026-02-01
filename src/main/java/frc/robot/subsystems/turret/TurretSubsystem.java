package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.devices.motor.Motor;
import frc.robot.shared.subsystems.AbstractSetAndSeekSubsystem;
import frc.robot.subsystems.turret.config.TurretSubsystemConfig;
import frc.robot.subsystems.turret.devices.TurretAbsoluteEncoder;
import frc.robot.subsystems.turret.devices.TurretMotor;
import frc.robot.subsystems.turret.devices.TurretSimMotor;

/**
 * Turret subsystem with a single profiled motor. Exposes the set-and-seek API so callers can drive to angles in degrees while the superclass handles
 * motion profiling, limits, and logging.
 */
public class TurretSubsystem extends AbstractSetAndSeekSubsystem<TurretSubsystemConfig> {
    private static Motor buildMotor(TurretSubsystemConfig config) {
        if (!config.enabled) {
            return null;
        }

        return RobotBase.isReal()
                ? TurretMotor.create(config.turretMotorConfig)
                : TurretSimMotor.create(
                        config.turretMotorConfig,
                        config.getMaximumVelocityDegreesPerSecondSupplier(),
                        config.getMaximumAccelerationDegreesPerSecondSquaredSupplier());
    }

    private final TurretAbsoluteEncoder absoluteEncoder;

    private boolean                     absoluteSeeded;

    private boolean                     absoluteSeedWarningIssued;

    private double                      lastAbsoluteAngleDegrees = Double.NaN;

    /**
     * Builds the turret subsystem with a single SparkMax-driven motor and default motion profile values.
     *
     * @param config turret configuration bundle loaded from JSON; angles are expressed in degrees
     */
    public TurretSubsystem(TurretSubsystemConfig config) {
        this(config, buildMotor(config));
    }

    private TurretSubsystem(TurretSubsystemConfig config, Motor motor) {
        super(config, motor);
        absoluteEncoder = buildAbsoluteEncoder(config);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (absoluteEncoder != null) {
            absoluteEncoder.logRawAngles(log);
            absoluteEncoder.logStatus(log);
            log.recordOutput("absoluteEncoderSeeded", absoluteSeeded);
            log.recordOutput("absoluteEncoderLastAngleDegrees", lastAbsoluteAngleDegrees);
            attemptAbsoluteSeed();
        }
    }

    public void logAbsoluteEncoderSnapshot() {
        if (absoluteEncoder == null) {
            log.warning("Absolute encoder snapshot requested, but absolute encoders are disabled.");
            return;
        }
        absoluteEncoder.logSnapshot(log);
    }

    private TurretAbsoluteEncoder buildAbsoluteEncoder(TurretSubsystemConfig config) {
        if (!config.enabled) {
            return null;
        }
        if (!config.absoluteEncoderConfig.enabled) {
            return null;
        }
        if (!isReal()) {
            return null;
        }
        log.info("Initializing turret absolute encoders for EasyCRT seeding.");
        return new TurretAbsoluteEncoder(
                config.absoluteEncoderConfig,
                config.getMinimumSetpointDegreesSupplier(),
                config.getMaximumSetpointDegreesSupplier());
    }

    private void attemptAbsoluteSeed() {
        if (absoluteSeeded || isSubsystemDisabled()) {
            return;
        }
        absoluteEncoder.solveMechanismAngle().ifPresentOrElse(
                angle -> {
                    double angleRadians = angle.in(edu.wpi.first.units.Units.Radians);
                    if (applyAbsoluteSeed(angleRadians)) {
                        lastAbsoluteAngleDegrees = angle.in(edu.wpi.first.units.Units.Degrees);
                        absoluteSeeded = true;
                        absoluteSeedWarningIssued = false;
                        settleAtCurrentPosition();
                        log.info(String.format("Turret absolute seed applied: %.3f deg",
                                lastAbsoluteAngleDegrees));
                    } else {
                        log.warning("Turret absolute seed failed: motor controller unavailable.");
                    }
                },
                () -> {
                    if (!absoluteSeedWarningIssued) {
                        log.warning("Turret absolute seed failed: EasyCRT could not resolve angle.");
                        absoluteSeedWarningIssued = true;
                    }
                });
    }

    private boolean applyAbsoluteSeed(double angleRadians) {
        if (motor instanceof TurretMotor turretMotor) {
            turretMotor.setRelativePositionRadians(angleRadians);
            return true;
        }
        if (motor.getMotor() != null) {
            motor.getMotor().getEncoder().setPosition(angleRadians);
            return true;
        }
        return false;
    }
}
