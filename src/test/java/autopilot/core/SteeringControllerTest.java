package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class SteeringControllerTest {

    private static final double NORMAL_SPEED_KMH = 80.0;
    private static final double HIGH_SPEED_KMH = 130.0;
    private static final double VALID_ANGLE_DEGREES = 30.0;
    private static final double REQUESTED_100_DEGREES = 100.0;
    private static final double EXPECTED_HIGH_SPEED_CLAMPED_TARGET = 90.0;

    private SteeringController steeringController;

    @BeforeEach
    void setUp() {
        steeringController = new SteeringController();
    }

    @ParameterizedTest
    @CsvSource({
            "0.0,    50.0,  0.0,   0.0",
            "30.0,   80.0,  30.0,  30.0",
            "-45.0,  80.0, -45.0, -45.0",
            "60.0,   80.0,  60.0,  60.0",
            "100.0,  80.0,  100.0, 60.0",
            "-100.0, 80.0, -100.0,-60.0",
            "500.0,  120.0, 450.0, 60.0",
            "-500.0, 120.0,-450.0,-60.0",
            "200.0,  120.1, 90.0,  60.0",
            "-200.0, 150.0,-90.0, -60.0"
    })
    void update_SingleTickFromZeroAngle_ClampsTargetAndMovesWithinTickLimit(
            double requestedAngleDegrees,
            double speedKmh,
            double expectedTargetAngleDegrees,
            double expectedCurrentAngleDegrees) {
        double actualCurrentAngle = steeringController.update(requestedAngleDegrees, speedKmh);

        assertEquals(expectedCurrentAngleDegrees, actualCurrentAngle);
        assertEquals(expectedTargetAngleDegrees, steeringController.getTargetAngleDegrees());
    }

    @Test
    void update_MultipleTicksAtHighSpeed_ReachesHighSpeedClampedTargetAngle() {
        steeringController.update(REQUESTED_100_DEGREES, HIGH_SPEED_KMH);

        double secondTickAngle = steeringController.update(REQUESTED_100_DEGREES, HIGH_SPEED_KMH);

        assertEquals(EXPECTED_HIGH_SPEED_CLAMPED_TARGET, secondTickAngle);
    }

    @ParameterizedTest
    @CsvSource({
            "40.0,   0.0",
            "-40.0,  0.0",
            "100.0,  0.0",
            "-100.0, 0.0"
    })
    void selfCenter_AfterSingleTickMovement_MovesTowardZeroWithinTickLimit(
            double initialRequestedAngle,
            double expectedAngleAfterOneSelfCenter) {
        steeringController.update(initialRequestedAngle, NORMAL_SPEED_KMH);

        double actualAngle = steeringController.selfCenter();

        assertEquals(expectedAngleAfterOneSelfCenter, actualAngle);
        assertEquals(0.0, steeringController.getTargetAngleDegrees());
    }

    @Test
    void selfCenter_FromAngleBeyondSingleTickStep_ReducesAngleByMaxStepPerTick() {
        steeringController.update(REQUESTED_100_DEGREES, NORMAL_SPEED_KMH);
        steeringController.update(REQUESTED_100_DEGREES, NORMAL_SPEED_KMH);

        double angleAfterFirstSelfCenter = steeringController.selfCenter();

        assertEquals(40.0, angleAfterFirstSelfCenter);
    }

    @ParameterizedTest
    @ValueSource(doubles = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_NonFiniteAngle_ThrowsIllegalArgumentException(double invalidAngleDegrees) {
        assertThrows(
                IllegalArgumentException.class,
                () -> steeringController.update(invalidAngleDegrees, NORMAL_SPEED_KMH)
        );
    }

    @ParameterizedTest
    @ValueSource(doubles = {-0.1, -50.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_InvalidSpeed_ThrowsIllegalArgumentException(double invalidSpeedKmh) {
        assertThrows(
                IllegalArgumentException.class,
                () -> steeringController.update(VALID_ANGLE_DEGREES, invalidSpeedKmh)
        );
    }
}