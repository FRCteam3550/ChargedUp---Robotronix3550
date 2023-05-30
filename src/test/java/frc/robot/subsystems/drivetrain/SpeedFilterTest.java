package frc.robot.subsystems.drivetrain;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import java.util.function.DoubleSupplier;

public class SpeedFilterTest {
    private class FakeTimer implements DoubleSupplier {
        private double m_currentTimeS = 0;

        void setTime(double timeS) {
            m_currentTimeS = timeS;
        }

        @Override
        public double getAsDouble() {
            return m_currentTimeS;
        }
    }

    private SpeedSetPoint setPoint(double setPoint) {
        return new SpeedSetPoint(setPoint, setPoint, setPoint);
    }

    @Test
    void whenWithinDeadBandThenReturn0() {
        final var DEADBAND = 0.1;
        final var speedFilter = new SpeedFilter(DEADBAND, 1, 1, Double.NaN, 1, 1, new FakeTimer());

        assertEquals(setPoint(0), speedFilter.filter(setPoint(0)));
        assertEquals(setPoint(0), speedFilter.filter(setPoint(0.1)));
        assertEquals(setPoint(0), speedFilter.filter(setPoint(0.09)));
    }

    @Test
    void whenOutsideDeadBandThenReturnAsIs() {
        final var DEADBAND = 0.1;
        final var speedFilter = new SpeedFilter(DEADBAND, 1, 1, Double.NaN, 1, 1, new FakeTimer());

        assertEquals(setPoint(0.11), speedFilter.filter(setPoint(0.11)));
        assertEquals(setPoint(1), speedFilter.filter(setPoint(1)));
    }
    
    @Test
    void whenPreciseModeThenReturnApplyLimitAndNotSquareInputs() {
        final var PRECISE_MODE_LIMIT = 0.3;
        final var speedFilter = new SpeedFilter(0.1, 1, PRECISE_MODE_LIMIT, Double.NaN, 1, 1, new FakeTimer());

        assertEquals(setPoint(0.3), speedFilter.filter(setPoint(1)));
        assertEquals(setPoint(0.15), speedFilter.filter(setPoint(0.5)));
    }
    
    @Test
    void whenFastModeThenReturnApplyLimitAndSquareInputs() {
        final var FAST_MODE_LIMIT = 0.8;
        final var speedFilter = new SpeedFilter(0.1, FAST_MODE_LIMIT, 0.3, Double.NaN, 1, 1, new FakeTimer());
        speedFilter.switchToFastMode();

        assertEquals(setPoint(0.8 * 0.8), speedFilter.filter(setPoint(1)));
        assertEquals(setPoint(0.4 * 0.4), speedFilter.filter(setPoint(0.5)));
    }

    @Test
    void whenTranslationAboveAccelerationLimitThenReturnLimitInput() {
        final var ACCELERATION_LIMIT = 0.3; // 30% per second
        final var fakeTimer = new FakeTimer();
        
        var speedFilter = new SpeedFilter(0.01, 1, 1, ACCELERATION_LIMIT, 1, 1, fakeTimer);
        assertEquals(new SpeedSetPoint(0.3, 0, 0), speedFilter.filter(new SpeedSetPoint(0.4, 0, 0)));

        speedFilter = new SpeedFilter(0.01, 1, 1, ACCELERATION_LIMIT, 1, 1, fakeTimer);
        assertEquals(new SpeedSetPoint(0, 0.3, 0), speedFilter.filter(new SpeedSetPoint(0, 0.4, 0)));

        speedFilter = new SpeedFilter(0.01, 1, 1, ACCELERATION_LIMIT, 1, 1, fakeTimer);
        // Call the filter at the begining with a set point within 30%/s (no adjustment)
        speedFilter.filter(new SpeedSetPoint(0.1, 0.2, 0));
        // Go forward in time half a second
        fakeTimer.setTime(0.5); 
        // Set a point above the acceleration limit (sqrt(0.15^2 + 0.2^2) / 0.5sec = 25% / 0.5sec = 50% per second)
        // Should reduce input variation per 30%/50% = 0.6
        assertEquals(new SpeedSetPoint(0.1 + 0.6 * 0.15, 0.2 + 0.6 * 0.2, 0), speedFilter.filter(new SpeedSetPoint(0.25, 0.4, 0)));
    }    

    @Test
    void whenTranslationBelowAccelerationLimitThenReturnDoNothing() {
        final var ACCELERATION_LIMIT = 0.3; // 30% per second
        final var fakeTimer = new FakeTimer();
        
        var speedFilter = new SpeedFilter(0.01, 1, 1, ACCELERATION_LIMIT, 1, 1, fakeTimer);
        assertEquals(new SpeedSetPoint(0.25, 0, 0), speedFilter.filter(new SpeedSetPoint(0.25, 0, 0)));

        speedFilter = new SpeedFilter(0.01, 1, 1, ACCELERATION_LIMIT, 1, 1, fakeTimer);
        assertEquals(new SpeedSetPoint(0, 0.25, 0), speedFilter.filter(new SpeedSetPoint(0, 0.25, 0)));

        speedFilter = new SpeedFilter(0.01, 1, 1, ACCELERATION_LIMIT, 1, 1, fakeTimer);
        // Call the filter at the begining with 0 set point
        speedFilter.filter(new SpeedSetPoint(0, 0, 0));
        // Go forward in time half a second
        fakeTimer.setTime(0.5); 
        assertEquals(new SpeedSetPoint(0.05, 0.06, 0), speedFilter.filter(new SpeedSetPoint(0.05, 0.06, 0)));
    }

    @Test
    void whenRotationAboveAccelerationLimitThenDoNothing() {
        final var ACCELERATION_LIMIT = 0.3; // 30% per second
        final var fakeTimer = new FakeTimer();

        var speedFilter = new SpeedFilter(0.01, 1, 1, ACCELERATION_LIMIT, 1, 1, fakeTimer);
        assertEquals(new SpeedSetPoint(0.15, 0.15, 0.4), speedFilter.filter(new SpeedSetPoint(0.15, 0.15, 0.4)));

        speedFilter = new SpeedFilter(0.01, 1, 1, ACCELERATION_LIMIT, 1, 1, fakeTimer);
        // Call the filter at the begining with 0 set point
        speedFilter.filter(new SpeedSetPoint(0, 0, 0));
        // Go forward in time half a second
        fakeTimer.setTime(0.5); 
        assertEquals(new SpeedSetPoint(0.1, 0.1, 0.6), speedFilter.filter(new SpeedSetPoint(0.1, 0.1, 0.6)));
    }

    @Test
    void whenConversionRatioThenApplyConversion() {
        final var TRANSLATION_CONVERSION = 3;
        final var ROTATION_CONVERSION = 6;
        final var speedFilter = new SpeedFilter(0.1, 1, 1, Double.NaN, TRANSLATION_CONVERSION, ROTATION_CONVERSION, new FakeTimer());

        assertEquals(new SpeedSetPoint(0.5 * 3, 0.8 * 3, 0.2 * 6), speedFilter.filter(new SpeedSetPoint(0.5, 0.8, 0.2)));
    }
}
