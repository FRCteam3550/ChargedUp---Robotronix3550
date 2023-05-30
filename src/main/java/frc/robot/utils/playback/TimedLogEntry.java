package frc.robot.utils.playback;

public class TimedLogEntry {
    public final double elapsedTimeSeconds;
    public final double[] data;

    public TimedLogEntry(double elapsedTimeSeconds, double... data) {
        this.elapsedTimeSeconds = elapsedTimeSeconds;
        this.data = data;
    }
}
