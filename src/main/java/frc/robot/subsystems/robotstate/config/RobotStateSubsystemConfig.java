package frc.robot.subsystems.robotstate.config;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.shared.config.AbstractConfig;

/**
 * Configuration bundle for the Robot State subsystem.
 * <p>
 * These values control how odometry and vision measurements are fused into a single pose estimate. All tunables are mirrored to AdvantageKit-backed
 * dashboard entries so they can be adjusted without a redeploy when not attached to FMS.
 * </p>
 */
public class RobotStateSubsystemConfig extends AbstractConfig {

    /**
     * Baseline standard deviation for odometry translation in meters.
     */
    public double  odometryTranslationStandardDeviationMeters = 0.02;

    /**
     * Baseline standard deviation for odometry rotation in radians.
     */
    public double  odometryRotationStandardDeviationRadians   = Units.degreesToRadians(1.0);

    /**
     * Baseline standard deviation for vision translation in meters.
     */
    public double  visionTranslationStandardDeviationMeters   = 0.5;

    /**
     * Baseline standard deviation for vision rotation in radians.
     */
    public double  visionRotationStandardDeviationRadians     = Units.degreesToRadians(10.0);

    /**
     * Maximum age in seconds for a vision measurement to be fused.
     */
    public double  visionMeasurementMaxAgeSeconds             = 0.25;

    /**
     * Blend factor applied when mixing odometry and vision poses (0 = odometry only, 1 = vision only).
     */
    public double  visionBlendFactor                          = 0.15;

    /**
     * Enables or disables vision fusion in the Robot State subsystem.
     */
    public boolean enableVisionFusion                         = true;

    /**
     * Returns the baseline odometry standard deviations.
     * <p>
     * Use this when configuring pose estimators that expect x/y translation in meters and rotation in radians.
     * </p>
     *
     * @return 3x1 matrix of odometry standard deviations in meters and radians
     */
    public Matrix<N3, N1> getOdometryStandardDeviations() {
        return VecBuilder.fill(
                readTunableNumber("odometryTranslationStandardDeviationMeters", odometryTranslationStandardDeviationMeters),
                readTunableNumber("odometryTranslationStandardDeviationMeters", odometryTranslationStandardDeviationMeters),
                readTunableNumber("odometryRotationStandardDeviationRadians", odometryRotationStandardDeviationRadians));
    }

    /**
     * Returns the baseline vision standard deviations.
     * <p>
     * Use this when configuring pose estimators that expect x/y translation in meters and rotation in radians.
     * </p>
     *
     * @return 3x1 matrix of vision standard deviations in meters and radians
     */
    public Matrix<N3, N1> getVisionStandardDeviations() {
        return VecBuilder.fill(
                readTunableNumber("visionTranslationStandardDeviationMeters", visionTranslationStandardDeviationMeters),
                readTunableNumber("visionTranslationStandardDeviationMeters", visionTranslationStandardDeviationMeters),
                readTunableNumber("visionRotationStandardDeviationRadians", visionRotationStandardDeviationRadians));
    }

    /**
     * Returns the maximum allowable age for vision measurements.
     * <p>
     * Older measurements are ignored to avoid blending stale camera data into the pose estimate.
     * </p>
     *
     * @return maximum vision measurement age in seconds
     */
    public double getVisionMeasurementMaxAgeSeconds() {
        return readTunableNumber("visionMeasurementMaxAgeSeconds", visionMeasurementMaxAgeSeconds);
    }

    /**
     * Returns the blend factor for fusing odometry with vision.
     * <p>
     * Values closer to 0 trust odometry more, values closer to 1 trust vision more.
     * </p>
     *
     * @return blend factor from 0 to 1
     */
    public double getVisionBlendFactor() {
        return readTunableNumber("visionBlendFactor", visionBlendFactor);
    }

    /**
     * Returns whether vision fusion should be enabled.
     * <p>
     * Disable this when running without cameras or while diagnosing vision issues.
     * </p>
     *
     * @return true when vision fusion is enabled
     */
    public boolean getEnableVisionFusion() {
        return readTunableBoolean("enableVisionFusion", enableVisionFusion);
    }
}
