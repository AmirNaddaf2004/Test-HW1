package autopilot.core;

import autopilot.exception.IllegalGearChangeException;
import autopilot.exception.SensorFailureException;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.List;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class GearBoxTest {

    private static final double ZERO_SPEED_KMH = 0.0;
    private static final double INVALID_NEGATIVE_SPEED_KMH = -10.0;

    @ParameterizedTest
    @MethodSource("validGearTransitionsProvider")
    void changeTo_ValidGearTransition_UpdatesCurrentGearToTarget(
            List<GearBox.Gear> setupTransitionsAtZeroSpeed,
            double transitionSpeedKmh,
            GearBox.Gear targetGear) {
        GearBox gearBox = new GearBox(() -> ZERO_SPEED_KMH);
        for (GearBox.Gear stepGear : setupTransitionsAtZeroSpeed) {
            gearBox.changeTo(stepGear);
        }
        gearBox.setSpeedSensor(() -> transitionSpeedKmh);

        gearBox.changeTo(targetGear);

        assertEquals(targetGear, gearBox.getCurrentGear());
    }

    private static Stream<Arguments> validGearTransitionsProvider() {
        return Stream.of(
                Arguments.of(List.of(), 0.0, GearBox.Gear.P),
                Arguments.of(List.of(), INVALID_NEGATIVE_SPEED_KMH, GearBox.Gear.P),
                Arguments.of(List.of(), 0.0, GearBox.Gear.R),
                Arguments.of(List.of(), 4.9, GearBox.Gear.R),
                Arguments.of(List.of(GearBox.Gear.R), 50.0, GearBox.Gear.R),
                Arguments.of(List.of(GearBox.Gear.R), 0.0, GearBox.Gear.D),
                Arguments.of(List.of(GearBox.Gear.R), 15.0, GearBox.Gear.N),
                Arguments.of(List.of(GearBox.Gear.R, GearBox.Gear.N), 20.0, GearBox.Gear.N),
                Arguments.of(List.of(GearBox.Gear.R, GearBox.Gear.N), 20.0, GearBox.Gear.D),
                Arguments.of(List.of(GearBox.Gear.R, GearBox.Gear.D), 50.0, GearBox.Gear.D),
                Arguments.of(List.of(GearBox.Gear.R, GearBox.Gear.D), 0.0, GearBox.Gear.R)
        );
    }

    @ParameterizedTest
    @MethodSource("illegalGearTransitionsProvider")
    void changeTo_DisallowedGearTransition_ThrowsAndKeepsCurrentGear(
            List<GearBox.Gear> setupTransitionsAtZeroSpeed,
            double transitionSpeedKmh,
            GearBox.Gear targetGear) {
        GearBox gearBox = new GearBox(() -> ZERO_SPEED_KMH);
        for (GearBox.Gear stepGear : setupTransitionsAtZeroSpeed) {
            gearBox.changeTo(stepGear);
        }
        gearBox.setSpeedSensor(() -> transitionSpeedKmh);
        GearBox.Gear originalGear = gearBox.getCurrentGear();

        assertThrows(
                IllegalGearChangeException.class,
                () -> gearBox.changeTo(targetGear)
        );
        assertEquals(originalGear, gearBox.getCurrentGear());
    }

    private static Stream<Arguments> illegalGearTransitionsProvider() {
        return Stream.of(
                Arguments.of(List.of(), 5.0, GearBox.Gear.R),
                Arguments.of(List.of(), 10.0, GearBox.Gear.R),
                Arguments.of(List.of(), 0.0, GearBox.Gear.N),
                Arguments.of(List.of(), 0.0, GearBox.Gear.D),
                Arguments.of(List.of(GearBox.Gear.R), 0.1, GearBox.Gear.D),
                Arguments.of(List.of(GearBox.Gear.R), 0.0, GearBox.Gear.P),
                Arguments.of(List.of(GearBox.Gear.R, GearBox.Gear.N), 0.0, GearBox.Gear.R),
                Arguments.of(List.of(GearBox.Gear.R, GearBox.Gear.N), 0.0, GearBox.Gear.P),
                Arguments.of(List.of(GearBox.Gear.R, GearBox.Gear.D), 0.1, GearBox.Gear.R),
                Arguments.of(List.of(GearBox.Gear.R, GearBox.Gear.D), 0.0, GearBox.Gear.N),
                Arguments.of(List.of(GearBox.Gear.R, GearBox.Gear.D), 0.0, GearBox.Gear.P)
        );
    }

    @ParameterizedTest
    @ValueSource(doubles = {-0.1, -10.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void changeTo_InvalidSpeedFromSensor_ThrowsIllegalArgumentException(double invalidSpeedKmh) {
        GearBox gearBox = new GearBox(() -> invalidSpeedKmh);

        assertThrows(
                IllegalArgumentException.class,
                () -> gearBox.changeTo(GearBox.Gear.R)
        );
    }

    @Test
    void changeTo_WhenSpeedSensorFails_PropagatesSensorFailureException() {
        GearBox gearBox = new GearBox(() -> {
            throw new SensorFailureException("Speed sensor failure");
        });

        assertThrows(
                SensorFailureException.class,
                () -> gearBox.changeTo(GearBox.Gear.R)
        );
    }

    @Test
    void changeTo_NullTargetGear_ThrowsNullPointerException() {
        GearBox gearBox = new GearBox(() -> ZERO_SPEED_KMH);

        assertThrows(
                NullPointerException.class,
                () -> gearBox.changeTo(null)
        );
    }
}
