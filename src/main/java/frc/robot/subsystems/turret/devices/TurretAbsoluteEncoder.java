package frc.robot.subsystems.turret.devices;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import frc.robot.shared.logging.Logger;
import frc.robot.subsystems.turret.config.TurretAbsoluteEncoderConfig;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

/**
 * An absolute encoder mechanism for the turret based on a pair of absolute
 * encoders connected to the turret with asymmetric idlers.  The turret
 * absolute position is resolved via the CRT algorithm implemmented in EasyCRT.
 * <p>
 * This mechanism is very sensitive to latency differences in encoder reading
 * so it is only intended to be used when the turret is stationary.  Ideally,
 * the turret subsystem will read the position at startup and then rely
 * entirely on relative encoders in the turret drive motor.
 * </p>
 */
public class TurretAbsoluteEncoder {

    private static int mapStatusCode(String status) {
        return switch (status) {
            case "OK" -> 1;
            case "NO_SOLUTION" -> 2;
            case "AMBIGUOUS" -> 3;
            case "INVALID_CONFIG" -> 4;
            default -> 0;
        };
    }

    private static double wrapRotations(double rotations) {
        return MathUtil.inputModulus(rotations, 0.0, 1.0);
    }

    private final TurretAbsoluteEncoderConfig config;

    private final Supplier<Double>            minimumDegreesSupplier;

    private final Supplier<Double>            maximumDegreesSupplier;

    /**
     * In most of the methods the logger is provided by the subsystem.  This
     * one is used in the case of a failure during setup / refresh.
     */
    private final Logger                      log;

    private final SparkAbsoluteEncoder        encoder1;

    private final SparkMax                    encoder1SparkMax;

    private final SparkAbsoluteEncoder        encoder2;

    private final SparkMax                    encoder2SparkMax;

    /** The API we use to get status and position from. */
    private final EasyCRT                     easyCrt;

    private final EasyCRTConfig               easyCrtConfig;

    public TurretAbsoluteEncoder(
            TurretAbsoluteEncoderConfig config,
            Supplier<Double> minimumDegreesSupplier,
            Supplier<Double> maximumDegreesSupplier) {
        this.config                 = config;
        this.minimumDegreesSupplier = minimumDegreesSupplier;
        this.maximumDegreesSupplier = maximumDegreesSupplier;
        this.log                    = Logger.getInstance(getClass(), config.verbose);

        encoder1SparkMax = createSpark(config.getEncoder1SparkMaxCanIdSupplier().get());
        encoder2SparkMax = createSpark(config.getEncoder2SparkMaxCanIdSupplier().get());

        encoder1      = encoder1SparkMax.getAbsoluteEncoder();
        encoder2      = encoder2SparkMax.getAbsoluteEncoder();

        easyCrtConfig = new EasyCRTConfig(this::getEncoder1Angle, this::getEncoder2Angle);
        easyCrt       = new EasyCRT(easyCrtConfig);
        refreshEasyCrtConfig();
    }

    public Optional<Angle> solveMechanismAngle() {
        refreshEasyCrtConfig();
        return easyCrt.getAngleOptional();
    }

    public Angle getEncoder1Angle() {
        return Rotations.of(wrapRotations(encoder1.getPosition()));
    }

    public Angle getEncoder2Angle() {
        return Rotations.of(wrapRotations(encoder2.getPosition()));
    }

    public String getLastStatus() {
        return easyCrt.getLastStatus();
    }

    public double getLastErrorRotations() {
        return easyCrt.getLastErrorRotations();
    }

    public int getLastIterations() {
        return easyCrt.getLastIterations();
    }

    public void logRawAngles(Logger turretLog) {
        turretLog.recordOutput("absoluteEncoder1/rawDegrees", getEncoder1Angle().in(Degrees));
        turretLog.recordOutput("absoluteEncoder2/rawDegrees", getEncoder2Angle().in(Degrees));
    }

    public void logStatus(Logger turretLog) {
        turretLog.recordOutput("absoluteEncoderEasyCrt/statusCode", mapStatusCode(getLastStatus()));
        turretLog.recordOutput("absoluteEncoderEasyCrt/lastErrorRotations", getLastErrorRotations());
        turretLog.recordOutput("absoluteEncoderEasyCrt/lastIterations", getLastIterations());
    }

    public void logSnapshot(Logger turretLog) {
        Angle encoder1Angle = getEncoder1Angle();
        Angle encoder2Angle = getEncoder2Angle();
        Optional<Angle> solvedAngle = solveMechanismAngle();

        turretLog.recordOutput("absoluteEncoderSnapshot/encoder1Degrees", encoder1Angle.in(Degrees));
        turretLog.recordOutput("absoluteEncoderSnapshot/encoder2Degrees", encoder2Angle.in(Degrees));
        turretLog.recordOutput("absoluteEncoderSnapshot/solvedDegrees",
                solvedAngle.map(angle -> angle.in(Degrees)).orElse(Double.NaN));
        logStatus(turretLog);

        log.info(String.format(
                "Absolute encoder snapshot: enc1=%.3f deg, enc2=%.3f deg, solved=%.3f deg, status=%s, errorRot=%.5f, iterations=%d",
                encoder1Angle.in(Degrees),
                encoder2Angle.in(Degrees),
                solvedAngle.map(angle -> angle.in(Degrees)).orElse(Double.NaN),
                getLastStatus(),
                getLastErrorRotations(),
                getLastIterations()));
    }

    private SparkMax createSpark(int canId) {
        SparkMax spark = new SparkMax(canId, MotorType.kBrushless);
        SparkMaxConfig sparkConfig = new SparkMaxConfig();
        sparkConfig.apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);
        spark.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return spark;
    }

    private void refreshEasyCrtConfig() {
        double commonRatio       = config.getCommonDriveRatioSupplier().get();
        int driveGearTeeth       = config.getDriveGearTeethSupplier().get();
        double encoder1OffsetDeg = config.getEncoder1OffsetDegreesSupplier().get();
        int encoder1PinionTeeth  = config.getEncoder1PinionTeethSupplier().get();
        double encoder2OffsetDeg = config.getEncoder2OffsetDegreesSupplier().get();
        int encoder2PinionTeeth  = config.getEncoder2PinionTeethSupplier().get();
        double minDegrees        = minimumDegreesSupplier.get();
        double maxDegrees        = maximumDegreesSupplier.get();
        double matchToleranceDeg = config.getMatchToleranceDegreesSupplier().get();

        try {
            easyCrtConfig
                    .withCommonDriveGear(commonRatio, driveGearTeeth, encoder1PinionTeeth, encoder2PinionTeeth)
                    .withAbsoluteEncoderOffsets(Degrees.of(encoder1OffsetDeg), Degrees.of(encoder2OffsetDeg))
                    .withAbsoluteEncoderInversions(
                            config.getEncoder1InvertedSupplier().get(),
                            config.getEncoder2InvertedSupplier().get())
                    .withMechanismRange(Degrees.of(minDegrees), Degrees.of(maxDegrees))
                    .withMatchTolerance(Degrees.of(matchToleranceDeg));
        } catch (IllegalArgumentException ex) {
            log.warning("EasyCRT config rejected input: " + ex.getMessage());
        }
    }
}
