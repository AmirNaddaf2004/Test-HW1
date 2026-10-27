package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class AdaptiveCruiseTest {

    private static final double DEFAULT_TARGET_SPEED_KMH = 100.0;
    private static final double SPEED_TO_SAFE_DISTANCE_FACTOR = 0.5;
    private static final double TEST_SPEED_80_KMH = 80.0;
    private static final double VALID_DISTANCE_50_METERS = 50.0;

    private static final double DISTANCE_BELOW_SAFE_30_METERS = 30.0;
    private static final double DISTANCE_IN_HYSTERESIS_BAND_44_METERS = 44.0;
    private static final double DISTANCE_AT_RELEASE_THRESHOLD_48_METERS = 48.0;
    private static final double EXPECTED_MAX_ACCELERATION_STEP = 5.0;
    private static final double DELTA = 1e-6;

    private AdaptiveCruise adaptiveCruise;

    @BeforeEach
    void setUp() {
        adaptiveCruise = new AdaptiveCruise(DEFAULT_TARGET_SPEED_KMH);
    }

    @ParameterizedTest
    @ValueSource(doubles = {0.0, 20.0, 80.0, 120.5})
    void safeDistanceFor_ValidNonNegativeSpeed_ReturnsHalfOfSpeed(double speedKmh) {
        double expectedSafeDistance = speedKmh * SPEED_TO_SAFE_DISTANCE_FACTOR;

        double actualSafeDistance = adaptiveCruise.safeDistanceFor(speedKmh);

        assertEquals(expectedSafeDistance, actualSafeDistance, DELTA);
    }

    @ParameterizedTest
    @ValueSource(doubles = {-0.1, -50.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void safeDistanceFor_InvalidSpeed_ThrowsIllegalArgumentException(double invalidSpeedKmh) {
        assertThrows(
                IllegalArgumentException.class,
                () -> adaptiveCruise.safeDistanceFor(invalidSpeedKmh)
        );
    }

    @ParameterizedTest
    @CsvFileSource(resources = "/adaptive-cruise-update.csv", numLinesToSkip = 1)
    void update_InitialNonBrakingState_ReturnsExpectedDecision(
            double targetSpeedKmh,
            double currentSpeedKmh,
            double distanceMeters,
            AdaptiveCruise.Mode expectedMode,
            double expectedAmount) {
        AdaptiveCruise cruise = new AdaptiveCruise(targetSpeedKmh);

        AdaptiveCruise.Decision actualDecision = cruise.update(currentSpeedKmh, distanceMeters);

        assertEquals(expectedMode, actualDecision.mode());
        assertEquals(expectedAmount, actualDecision.amount(), DELTA);
    }

    @ParameterizedTest
    @CsvSource({
            "20.0,   EMERGENCY_BRAKE, 100.0",
            "30.0,   BRAKE,            25.0",
            "39.9,   BRAKE,             0.25",
            "40.0,   BRAKE,            10.0",
            "44.0,   BRAKE,            10.0",
            "47.999, BRAKE,            10.0",
            "48.0,   ACCELERATE,        5.0",
            "49.0,   ACCELERATE,        5.0"
    })
    void update_WhenAlreadyBraking_ReturnsExpectedDecisionAtDistanceThresholds(
            double distanceMeters,
            AdaptiveCruise.Mode expectedMode,
            double expectedAmount) {
        adaptiveCruise.update(TEST_SPEED_80_KMH, DISTANCE_BELOW_SAFE_30_METERS);

        AdaptiveCruise.Decision actualDecision = adaptiveCruise.update(
                TEST_SPEED_80_KMH, distanceMeters);

        assertEquals(expectedMode, actualDecision.mode());
        assertEquals(expectedAmount, actualDecision.amount(), DELTA);
    }

    @Test
    void update_AfterReleasingBrake_DoesNotBrakeAgainAboveSafeDistance() {
        adaptiveCruise.update(TEST_SPEED_80_KMH, DISTANCE_BELOW_SAFE_30_METERS);
        adaptiveCruise.update(TEST_SPEED_80_KMH, DISTANCE_AT_RELEASE_THRESHOLD_48_METERS);

        AdaptiveCruise.Decision actualDecision = adaptiveCruise.update(
                TEST_SPEED_80_KMH,
                DISTANCE_IN_HYSTERESIS_BAND_44_METERS
        );

        assertEquals(AdaptiveCruise.Mode.ACCELERATE, actualDecision.mode());
        assertEquals(EXPECTED_MAX_ACCELERATION_STEP, actualDecision.amount(), DELTA);
    }

    @ParameterizedTest
    @ValueSource(doubles = {-0.1, -50.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_InvalidSpeed_ThrowsIllegalArgumentException(double invalidSpeedKmh) {
        assertThrows(
                IllegalArgumentException.class,
                () -> adaptiveCruise.update(invalidSpeedKmh, VALID_DISTANCE_50_METERS)
        );
    }

    @ParameterizedTest
    @ValueSource(doubles = {-0.1, -50.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_InvalidDistance_ThrowsIllegalArgumentException(double invalidDistanceMeters) {
        assertThrows(
                IllegalArgumentException.class,
                () -> adaptiveCruise.update(TEST_SPEED_80_KMH, invalidDistanceMeters)
        );
    }
}
