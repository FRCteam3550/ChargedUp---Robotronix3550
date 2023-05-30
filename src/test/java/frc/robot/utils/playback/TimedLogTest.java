package frc.robot.utils.playback;

import org.junit.jupiter.api.Test;

import frc.robot.utils.SimpleTimer;

import org.junit.jupiter.api.AfterAll;
import static org.junit.jupiter.api.Assertions.*;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

public class TimedLogTest {
    private static final LoadDirectory DIRECTORY = LoadDirectory.Home;
    private static final String name = "mouvementAuto1Test";

    @AfterAll
    static void cleanup() throws IOException {
        // À la fin des tests, on supprime les fichiers que l'on a créé.
        Files
            .walk(Path.of(DIRECTORY.path))
            .filter((f) -> f.getFileName().toString().startsWith(name) && f.toString().endsWith(".csv"))
            .forEach((f) -> f.toFile().delete());
    }
    
    @Test
    void canRecordAndPlayData() throws InterruptedException {
        var fakeTimer = new SimpleTimer.FakeSimpleTimer();
        var recorder = TimedLog.startRecording(name, fakeTimer);

        fakeTimer.setNextTime(0);
        recorder.recordLogEntry(0.1, 0.2);    

        fakeTimer.setNextTime(0.01);
        recorder.recordLogEntry(0.3, 0.4);

        fakeTimer.setNextTime(0.02);
        recorder.recordLogEntry(0, 0);
        recorder.save();

        var reader = TimedLog.loadLastFileForName(DIRECTORY, name, fakeTimer);

        fakeTimer.setNextTime(0);
        assertArrayEquals(new double[]{0.1, 0.2}, reader.readLogEntry().get());

        fakeTimer.setNextTime(0.01);
        assertArrayEquals(new double[]{0.3, 0.4}, reader.readLogEntry().get());

        fakeTimer.setNextTime(0.03);
        assertTrue(reader.readLogEntry().isEmpty());
    }
}
