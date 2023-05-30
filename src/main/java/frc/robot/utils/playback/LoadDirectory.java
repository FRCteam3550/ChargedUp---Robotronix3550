package frc.robot.utils.playback;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Représente la liste des répértoires sur le robot à partir desquels on peut lire des enregistrements de données horodatées.
 */
public enum LoadDirectory {
    Home(Filesystem.getOperatingDirectory()),
    Deploy(Filesystem.getDeployDirectory());

    final String path;
    LoadDirectory(File directory) {
        path = directory.getAbsolutePath();
    }
}
