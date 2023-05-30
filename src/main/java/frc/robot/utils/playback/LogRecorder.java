package frc.robot.utils.playback;

public interface LogRecorder {
    /**
     * Enregistre les infos données avec le temps écoulé depuis le début de l'enregistrement.
     * Le temps écoulé commence lorsque l'on appelle la méthode {@link  #startRecording  startRecording}.
     * @param data Les données que l'on souhaite enregistrer pour le moment courant.
     */
    void recordLogEntry(double... data);
    /**
     * Sauve le journal sur fichier. Le répertoire est toujours le répertoire de l'utilisateur courant sur le robot.
     * Le nom sera une combinaison du nom du journal et de la date d'enregistrement.
     */
    void save();
}
