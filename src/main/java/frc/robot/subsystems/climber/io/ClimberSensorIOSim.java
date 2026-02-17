package frc.robot.subsystems.climber.io;

import java.util.function.DoubleSupplier;

/**
 * Simulation implementation of the climber Time of Flight sensors that returns configurable default values.
 * <p>
 * By default both sensors report 200 mm and are marked as connected so the subsystem can exercise its periodic logic during {@code simulateJava}
 * without crashing. Distance suppliers can be injected for test scenarios that need dynamic values.
 * </p>
 */
public class ClimberSensorIOSim implements ClimberSensorIO {

    /** Default simulated distance in millimeters. */
    private static final double DEFAULT_DISTANCE_MILLIMETERS = 200.0;

    /** Supplier for the simulated left sensor distance in millimeters. */
    private final DoubleSupplier leftDistanceSupplier;

    /** Supplier for the simulated right sensor distance in millimeters. */
    private final DoubleSupplier rightDistanceSupplier;

    /**
     * Creates a simulation sensor IO with default fixed distances.
     */
    public ClimberSensorIOSim() {
        this(() -> DEFAULT_DISTANCE_MILLIMETERS, () -> DEFAULT_DISTANCE_MILLIMETERS);
    }

    /**
     * Creates a simulation sensor IO with injectable distance suppliers for test scenarios.
     *
     * @param leftDistanceSupplier  supplier that returns the simulated left sensor distance in millimeters
     * @param rightDistanceSupplier supplier that returns the simulated right sensor distance in millimeters
     */
    public ClimberSensorIOSim(DoubleSupplier leftDistanceSupplier, DoubleSupplier rightDistanceSupplier) {
        this.leftDistanceSupplier  = leftDistanceSupplier;
        this.rightDistanceSupplier = rightDistanceSupplier;
    }

    @Override
    public void updateInputs(ClimberSensorIOInputs inputs) {
        inputs.leftDistanceMillimeters  = leftDistanceSupplier.getAsDouble();
        inputs.rightDistanceMillimeters = rightDistanceSupplier.getAsDouble();
        inputs.leftSensorConnected      = true;
        inputs.rightSensorConnected     = true;
    }
}
