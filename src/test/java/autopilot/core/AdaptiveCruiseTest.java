package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvFileSource;
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
    private static final double EXPECTED_PROPORTIONAL_BRAKE_25 = 25.0;
    private static final double EXPECTED_HYSTERESIS_BRAKE_PRESSURE = 10.0;
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

    @Test
    void update_WhenAlreadyBrakingAndDistanceBelowSafeDistance_AppliesProportionalBrake() {
        adaptiveCruise.update(TEST_SPEED_80_KMH, DISTANCE_BELOW_SAFE_30_METERS);

        AdaptiveCruise.Decision secondDecision = adaptiveCruise.update(
                TEST_SPEED_80_KMH,
                DISTANCE_BELOW_SAFE_30_METERS
        );

        assertEquals(AdaptiveCruise.Mode.BRAKE, secondDecision.mode());
        assertEquals(EXPECTED_PROPORTIONAL_BRAKE_25, secondDecision.amount(), DELTA);
    }

    @Test
    void update_WhenAlreadyBrakingAndDistanceBetweenSafeAndRelease_AppliesHysteresisBrakePressure() {
        adaptiveCruise.update(TEST_SPEED_80_KMH, DISTANCE_BELOW_SAFE_30_METERS);

        AdaptiveCruise.Decision hysteresisDecision = adaptiveCruise.update(
                TEST_SPEED_80_KMH,
                DISTANCE_IN_HYSTERESIS_BAND_44_METERS
        );

        assertEquals(AdaptiveCruise.Mode.BRAKE, hysteresisDecision.mode());
        assertEquals(EXPECTED_HYSTERESIS_BRAKE_PRESSURE, hysteresisDecision.amount(), DELTA);
    }

    @Test
    void update_WhenAlreadyBrakingAndDistanceReachesReleaseThreshold_ExitsBrakingAndAccelerates() {
        adaptiveCruise.update(TEST_SPEED_80_KMH, DISTANCE_BELOW_SAFE_30_METERS);

        AdaptiveCruise.Decision releasedDecision = adaptiveCruise.update(
                TEST_SPEED_80_KMH,
                DISTANCE_AT_RELEASE_THRESHOLD_48_METERS
        );

        assertEquals(AdaptiveCruise.Mode.ACCELERATE, releasedDecision.mode());
        assertEquals(EXPECTED_MAX_ACCELERATION_STEP, releasedDecision.amount(), DELTA);
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