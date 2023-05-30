package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

/**
 * A contract for classes using timers, so we can replace the default timers by a mocked one.
 */
public interface SimpleTimer {
    void start();
    double getTimeS();

    /**
     * Returns a real timer.
     */
    public static SimpleTimer create() {
        var timer = new Timer();
        return new SimpleTimer() {
            @Override
            public void start() {
                timer.start();
            }

            @Override
            public double getTimeS() {
                return timer.get();
            }
        };
    }

    /**
     * Returns a fake timer useful for tests.
     */
    public class FakeSimpleTimer implements SimpleTimer {
        private double m_nextTimeS = 0;

        public void setNextTime(double nextTimeS) {
            m_nextTimeS = nextTimeS;
        }

        @Override
        public void start() {
        }

        @Override
        public double getTimeS() {
            return m_nextTimeS;
        }
    }
}
