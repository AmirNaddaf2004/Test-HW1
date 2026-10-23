package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;

class AutoDriverTest {

    private static final double TARGET_SPEED_KMH = 80.0;
    private static final double DEFAULT_SPEED_KMH = 80.0;
    private static final double SAFE_OBSTACLE_DISTANCE_METERS = 100.0;
    private static final double CENTERED_LANE_OFFSET_METERS = 0.0;

    private static final double EXPECTED_CENTERED_STEERING_DEGREES = 0.0;
    private static final double EXPECTED_NO_BRAKE_PRESSURE = 0.0;
    private static final double EXPECTED_SAFE_MODE_BRAKE_PRESSURE = 30.0;
    private static final String SENSOR_FAILURE_MESSAGE = "Sensor read failure";

    private SpeedSensor defaultSpeedSensor;
    private ObstacleSensor defaultObstacleSensor;
    private LaneSensor defaultLaneSensor;

    @BeforeEach
    void setUp() {
        defaultSpeedSensor = () -> DEFAULT_SPEED_KMH;
        defaultObstacleSensor = () -> SAFE_OBSTACLE_DISTANCE_METERS;
        defaultLaneSensor = () -> CENTERED_LANE_OFFSET_METERS;
    }

    @Test
    void tick_NormalDrivingAtTargetSpeedAndCenteredLane_HoldsSpeedWithZeroBrakeAndSteering() {
        AutoDriver autoDriver = new AutoDriver(
                defaultSpeedSensor,
                defaultObstacleSensor,
                defaultLaneSensor,
                TARGET_SPEED_KMH
        );
        AutoDriver.DriveResult expectedResult = new AutoDriver.DriveResult(
                EXPECTED_CENTERED_STEERING_DEGREES,
                EXPECTED_NO_BRAKE_PRESSURE,
                AdaptiveCruise.Mode.HOLD,
                false,
                false
        );

        AutoDriver.DriveResult actualResult = autoDriver.tick(GearBox.Gear.D);

        assertEquals(expectedResult, actualResult);
    }

    @ParameterizedTest
    @CsvSource({
            "1.0,  -45.0",
            "-1.0,  45.0"
    })
    void tick_LaneOffsetExceedsThreshold_SteersInOppositeDirectionToCorrectLane(
            double laneOffsetMeters,
            double expectedSteeringAngleDegrees) {
        LaneSensor offsetLaneSensor = () -> laneOffsetMeters;
        AutoDriver autoDriver = new AutoDriver(
                defaultSpeedSensor,
                defaultObstacleSensor,
                offsetLaneSensor,
                TARGET_SPEED_KMH
        );

        AutoDriver.DriveResult actualResult = autoDriver.tick(GearBox.Gear.D);

        assertEquals(expectedSteeringAngleDegrees, actualResult.steeringAngleDegrees());
    }

    @ParameterizedTest
    @MethodSource("failingSensorConfigurationsProvider")
    void tick_WhenAnySensorFails_EntersSafeModeWithDefaultBraking(
            SpeedSensor speedSensor,
            ObstacleSensor obstacleSensor,
            LaneSensor laneSensor) {
        AutoDriver autoDriver = new AutoDriver(
                speedSensor,
                obstacleSensor,
                laneSensor,
                TARGET_SPEED_KMH
        );
        AutoDriver.DriveResult expectedSafeModeResult = new AutoDriver.DriveResult(
                EXPECTED_CENTERED_STEERING_DEGREES,
                EXPECTED_SAFE_MODE_BRAKE_PRESSURE,
                AdaptiveCruise.Mode.BRAKE,
                false,
                true
        );

        AutoDriver.DriveResult actualResult = autoDriver.tick(GearBox.Gear.D);

        assertEquals(expectedSafeModeResult, actualResult);
    }

    private static Stream<Arguments> failingSensorConfigurationsProvider() {
        SpeedSensor okSpeed = () -> DEFAULT_SPEED_KMH;
        ObstacleSensor okObstacle = () -> SAFE_OBSTACLE_DISTANCE_METERS;
        LaneSensor okLane = () -> CENTERED_LANE_OFFSET_METERS;

        SpeedSensor failingSpeed = () -> {
            throw new SensorFailureException(SENSOR_FAILURE_MESSAGE);
        };
        ObstacleSensor failingObstacle = () -> {
            throw new SensorFailureException(SENSOR_FAILURE_MESSAGE);
        };

        return Stream.of(
                Arguments.of(failingSpeed, okObstacle, okLane),
                Arguments.of(okSpeed, failingObstacle, okLane)
        );
    }
}