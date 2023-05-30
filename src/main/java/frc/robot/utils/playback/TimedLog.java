package frc.robot.utils.playback;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Optional;
import java.util.Scanner;
import java.util.StringJoiner;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.SimpleTimer;

/**
 * Représente un enregistrement de données numériques horodatées.
 * <p>
 * Ce peut être utilisé à des fins d'analyse, ou à des fins d'enregistrement et de replay de mouvements du robot.
 * </p>
 * <p>
 * Les fichiers sont enregistrés dans le répertoire {@link  edu.wpi.first.wpilibj.Filesystem#getOperatingDirectory  operatingDirectory}. Ils peuvent être chargé à partir du même répertoire, ou du répertoire de {@link  edu.wpi.first.wpilibj.Filesystem#getDeployDirectory  déploiement}.
 * </p>
 * <p>
 * Exemple d'enregistrement:
 * </p>
 * 
 * <pre>
 * 
 * private XBoxController gamepad;
 * private DifferentialDrive drive;
 * private LogRecorder recorder;
 * 
 * &#64;Override
 * public void teleopInit() {
 *   recorder = TimedLog.startRecording("autoMove1");
 * }
 * 
 * &#64;Override
 * public void teleopPeriodic() {
 *   recorder.recordLogEntry(gamepad.getLeftX(), gamepad.getLeftY());
 *   drive.arcadeDrive(gamepad.getLeftX(), gamepad.getLeftY());
 * }
 * 
 * </pre>
 *
 * <p>
 * Exemple de replay:
 * </p>
 * 
 * <pre>
 * 
 * private DifferentialDrive drive;
 * private LogReader reader;
 * 
 * &#64;Override
 * public void teleopInit() {
 *   reader = TimedLog.loadLastFileForName(LoadDirectory.Home, "autoMove1");
 *   reader.startReading();
 * }
 * 
 * &#64;Override
 * public void teleopPeriodic() {
 *   var toReplay = reader.readLogEntry();
 *   if (toReplay.isPresent()) {
 *     var inputs = toReplay.get();
 *     drive.arcadeDrive(inputs[0], inputs[1]);
 *   }
 * }
 * 
 * </pre>
 */
public class TimedLog implements LogReader, LogRecorder {
    private static final SimpleDateFormat DATE_FORMAT = new SimpleDateFormat("yyyy_MM_dd_hh_mm_ss");
    private final String name;
    private final SimpleTimer timer;
    private List<TimedLogEntry> entries = new ArrayList<>();
    private int currentLogEntry = 0;

    private TimedLog(String name, SimpleTimer timer) {
        this(name, new ArrayList<>(), timer);
    }

    private TimedLog(String name, List<TimedLogEntry> entries, SimpleTimer timer) {
        this.name = name;
        this.entries = entries;
        this.timer = timer;
    }

    /**
     * Crée un nouvel enregistrement vide et retourne un objet permettant d'y ajouter des entrées.
     * @param name le nom expliquant le but de ce journal. Par exemple, pour un journal enregistrant
     * une séquence de mouvements automatique, le nom de la séquence de mouvements.
     * @return un enregistreur pour un nouveau journal horodaté.
     */
    public static LogRecorder startRecording(String name) {
        return startRecording(name, SimpleTimer.create());
    }

    static LogRecorder startRecording(String name, SimpleTimer timer) {
        return new TimedLog(name, timer);
    }

    /**
     * Enregistre les infos données avec le temps écoulé depuis le début de l'enregistrement.
     * Le temps écoulé commence lorsque l'on appelle la méthode {@link  #startRecording  startRecording}.
     * @param data Les données que l'on souhaite enregistrer pour le moment courant.
     */
    public void recordLogEntry(double... data) {
        entries.add(new TimedLogEntry(timer.getTimeS(), data));
    }

    /**
     * Déclenche la minuterie pour lire le journal.
     */
    public void startReading() {
        timer.start();
        currentLogEntry = 0;
    }

    /**
     * Retourne les données du journal pour le moment courant, ou rien si la lecture est terminée.
     */
    public Optional<double[]> readLogEntry() {
        var elapsedTimeSeconds = timer.getTimeS();
        while (currentLogEntry < entries.size() - 1 && elapsedTimeSeconds > entries.get(currentLogEntry).elapsedTimeSeconds) {
            currentLogEntry += 1;
        }

        if (currentLogEntry < entries.size() - 1) {
            return Optional.of(entries.get(currentLogEntry).data);
        }
        return Optional.empty();
    }

    private File saveFile() {
        var fileName = String.format("%s_%s.csv", name, DATE_FORMAT.format(new Date()));
        return Path.of(LoadDirectory.Home.path, fileName).toFile();
    }

    /**
     * Sauve le journal sur fichier. Le répertoire est toujours le répertoire de l'utilisateur courant sur le robot.
     * Le nom sera une combinaison du nom du journal et de la date d'enregistrement.
     */
    public void save() {
        var file = saveFile();
        try (var writer = new PrintWriter(new BufferedWriter(new FileWriter(file)))) {
            for (var entry : entries) {
                var sj = new StringJoiner(",");
                sj.add(Double.toString(entry.elapsedTimeSeconds));
                for (var data : entry.data) {
                    sj.add(Double.toString(data));
                }
                writer.println(sj);
            }

            System.out.println(String.format("Sauvé l'enregistrement dans %s", file.getAbsolutePath()));
        }
        catch(IOException ioe) {
            var msg = String.format("Impossible de sauver l'enregistrement dans %s", file.getAbsolutePath());
            DriverStation.reportError(msg, false);
            System.out.println(msg);
        }
    }

    /**
     * Charge un enregistrement à relire à partir du répertoire donné, avec le nom de fichier donné.
     */
    public static LogReader loadFromFile(LoadDirectory dir, String fileName) {
        return loadFromFile(dir, fileName, SimpleTimer.create());
    }

    static LogReader loadFromFile(LoadDirectory dir, String fileName, SimpleTimer timer) {
        var file = Path.of(dir.path, fileName).toFile();

        try (var scanner = new Scanner(new FileReader(file))) {
            var result = new ArrayList<TimedLogEntry>();
            while (scanner.hasNextLine()) {
                var dataAsString = scanner.nextLine().split(",");
                var data = Stream.of(dataAsString).mapToDouble(Double::valueOf).toArray();
                var entry = new TimedLogEntry(
                   data[0],
                   Arrays.copyOfRange(data, 1, data.length)
                );
                result.add(entry);
            }

            var name = fileName.split("_")[0];
            System.out.println(String.format("Chargé l'enregistrement de %s", file.getAbsolutePath()));
            return new TimedLog(name, result, timer);
        }
        catch(IOException ioe) {
            throw new RuntimeException(String.format("Impossible de lire l'enregistrement de %s", file.getAbsolutePath()), ioe);
        }
    }

    /**
     * Charge le dernier journal enregistré avec le nom donné, à partir du répertoire donné.
     */
    public static LogReader loadLastFileForName(LoadDirectory dir, String name) {
        return loadFromFile(dir, name, SimpleTimer.create());
    }

    static LogReader loadLastFileForName(LoadDirectory dir, String name, SimpleTimer timer) {
        var prefix = name + "_";

        try {
            var files = Files
                .walk(Path.of(dir.path))
                .map((f) -> f.getFileName().toString())
                .filter((f) -> f.startsWith(prefix) && f.endsWith(".csv"))
                .sorted()
                .toList();
            if (files.size() == 0) {
                throw new IOException();
            }

            var lastFile = files.get(files.size() - 1);
            return loadFromFile(dir, lastFile, timer);
        }
        catch (IOException ioe) {
            throw new RuntimeException(String.format("Impossible de trouver un enregistrement avec le nom %s dans %s", name, dir.path), ioe);
        }
    }
}
