package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

class BrakeControllerTest {

    private static final double MAX_PRESSURE = 100.0;
    private static final double ABS_TRIGGER_PRESSURE = 95.0;
    private static final double NON_ABS_PRESSURE = 50.0;
    private static final double ABS_TRIGGER_SPEED_KMH = 60.0;
    private static final double NORMAL_SPEED_KMH = 40.0;
    private static final double ABS_HIGH_PULSE_PRESSURE = 90.0;
    private static final double ABS_LOW_PULSE_PRESSURE = 60.0;

    private BrakeController brakeController;

    @BeforeEach
    void setUp() {
        brakeController = new BrakeController();
    }

    @ParameterizedTest
    @CsvSource({
            "0.0,   50.0, D, 0.0",
            "50.0,  60.0, D, 50.0",
            "80.0,  60.0, D, 80.0",
            "90.0,  30.0, D, 90.0",
            "120.0, 20.0, R, 100.0",
            "85.0,  35.0, N, 90.0",
            "150.0, 80.0, D, 90.0"
    })
    void apply_ValidInputsOnInitialCall_ReturnsExpectedBoundedOrAbsPressure(
            double requestedPressure,
            double speedKmh,
            GearBox.Gear gear,
            double expectedPressure) {
        double actualPressure = brakeController.apply(requestedPressure, speedKmh, gear);

        assertEquals(expectedPressure, actualPressure);
    }

    @Test
    void apply_ConsecutiveAbsCalls_AlternatesBetweenHighAndLowPulse() {
        brakeController.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        double secondPulsePressure = brakeController.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        assertEquals(ABS_LOW_PULSE_PRESSURE, secondPulsePressure);
    }

    @Test
    void apply_NonAbsCallAfterHighAbsPulse_ResetsNextAbsPulseToHigh() {
        brakeController.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);
        brakeController.apply(NON_ABS_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        double pressureAfterReset = brakeController.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        assertEquals(ABS_HIGH_PULSE_PRESSURE, pressureAfterReset);
    }

    @Test
    void apply_WhenGearIsPark_ThrowsIllegalStateException() {
        assertThrows(
                IllegalStateException.class,
                () -> brakeController.apply(NON_ABS_PRESSURE, NORMAL_SPEED_KMH, GearBox.Gear.P)
        );
    }

    @Test
    void apply_WhenGearIsNull_ThrowsNullPointerException() {
        assertThrows(
                NullPointerException.class,
                () -> brakeController.apply(NON_ABS_PRESSURE, NORMAL_SPEED_KMH, null)
        );
    }

    @ParameterizedTest
    @ValueSource(doubles = {-0.1, -50.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void apply_InvalidPressure_ThrowsIllegalArgumentException(double invalidPressure) {
        assertThrows(
                IllegalArgumentException.class,
                () -> brakeController.apply(invalidPressure, NORMAL_SPEED_KMH, GearBox.Gear.D)
        );
    }

    @ParameterizedTest
    @ValueSource(doubles = {-0.1, -20.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void apply_InvalidSpeed_ThrowsIllegalArgumentException(double invalidSpeedKmh) {
        assertThrows(
                IllegalArgumentException.class,
                () -> brakeController.apply(NON_ABS_PRESSURE, invalidSpeedKmh, GearBox.Gear.D)
        );
    }

    @ParameterizedTest
    @EnumSource(value = GearBox.Gear.class, names = {"P"}, mode = EnumSource.Mode.EXCLUDE)
    void emergencyBrake_AllowedGears_SetsMaxPressureAndActivatesWarning(GearBox.Gear allowedGear) {
        brakeController.emergencyBrake(allowedGear);

        assertEquals(MAX_PRESSURE, brakeController.getPressure());
        assertTrue(brakeController.isWarningActive());
    }

    @Test
    void emergencyBrake_AfterHighAbsPulse_ResetsNextAbsPulseToHigh() {
        brakeController.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);
        brakeController.emergencyBrake(GearBox.Gear.D);

        double nextAbsPressure = brakeController.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        assertEquals(ABS_HIGH_PULSE_PRESSURE, nextAbsPressure);
    }

    @Test
    void emergencyBrake_WhenGearIsPark_ThrowsIllegalStateException() {
        assertThrows(
                IllegalStateException.class,
                () -> brakeController.emergencyBrake(GearBox.Gear.P)
        );
    }

    @Test
    void emergencyBrake_WhenGearIsNull_ThrowsNullPointerException() {
        assertThrows(
                NullPointerException.class,
                () -> brakeController.emergencyBrake(null)
        );
    }
}