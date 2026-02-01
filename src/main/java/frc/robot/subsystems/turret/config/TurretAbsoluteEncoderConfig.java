package frc.robot.subsystems.turret.config;

import java.util.function.Supplier;

import frc.robot.shared.config.AbstractConfig;

/**
 * Configuration bundle for the absolute encoders mechanism on the turret.
 * The mechanism consists of 2 REV Through-Bore encoders connected to
 * SparkMax motor controllers.
 * <p>
 * The encoders are expected to be configured as single-turn absolute
 * encoder mode.  Another expectation here is that the turret attached
 * to this mechanism is limited to 360° of total motion.
 * </p>
 */
public class TurretAbsoluteEncoderConfig extends AbstractConfig {

    /** Gear ratio between the mechanism and the gear driving both encoders. */
    public double  commonDriveRatio;

    /** Tooth count of the gear that drives both encoder pinions. */
    public int     driveGearTeeth;

    /** True when encoder 1 should be inverted. */
    public boolean encoder1Inverted;

    /** Offset for encoder 1 in degrees. */
    public double  encoder1OffsetDegrees;

    /** Tooth count of encoder 1 pinion. */
    public int     encoder1PinionTeeth;

    /** CAN ID for the first SparkMax hosting the absolute encoder. */
    public int     encoder1SparkMaxCanId;

    /** True when encoder 2 should be inverted. */
    public boolean encoder2Inverted;

    /** Offset for encoder 2 in degrees. */
    public double  encoder2OffsetDegrees;

    /** Tooth count of encoder 2 pinion. */
    public int     encoder2PinionTeeth;

    /** CAN ID for the second SparkMax hosting the absolute encoder. */
    public int     encoder2SparkMaxCanId;

    /** Match tolerance in degrees for EasyCRT. */
    public double  matchToleranceDegrees;

    public Supplier<Double> getCommonDriveRatioSupplier() {
        return () -> readTunableNumber("commonDriveRatio", commonDriveRatio);
    }

    public Supplier<Integer> getDriveGearTeethSupplier() {
        return () -> (int) readTunableNumber("driveGearTeeth", driveGearTeeth);
    }

    public Supplier<Boolean> getEncoder1InvertedSupplier() {
        return () -> readTunableBoolean("encoder1Inverted", encoder1Inverted);
    }

    public Supplier<Double> getEncoder1OffsetDegreesSupplier() {
        return () -> readTunableNumber("encoder1OffsetDegrees", encoder1OffsetDegrees);
    }

    public Supplier<Integer> getEncoder1PinionTeethSupplier() {
        return () -> (int) readTunableNumber("encoder1PinionTeeth", encoder1PinionTeeth);
    }

    public Supplier<Integer> getEncoder1SparkMaxCanIdSupplier() {
        return () -> (int) readTunableNumber("encoder1SparkMaxCanId", encoder1SparkMaxCanId);
    }

    public Supplier<Boolean> getEncoder2InvertedSupplier() {
        return () -> readTunableBoolean("encoder2Inverted", encoder2Inverted);
    }

    public Supplier<Double> getEncoder2OffsetDegreesSupplier() {
        return () -> readTunableNumber("encoder2OffsetDegrees", encoder2OffsetDegrees);
    }

    public Supplier<Integer> getEncoder2PinionTeethSupplier() {
        return () -> (int) readTunableNumber("encoder2PinionTeeth", encoder2PinionTeeth);
    }

    public Supplier<Integer> getEncoder2SparkMaxCanIdSupplier() {
        return () -> (int) readTunableNumber("encoder2SparkMaxCanId", encoder2SparkMaxCanId);
    }

    public Supplier<Double> getMatchToleranceDegreesSupplier() {
        return () -> readTunableNumber("matchToleranceDegrees", matchToleranceDegrees);
    }
}
