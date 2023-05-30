package frc.robot.utils.camestimate;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class LimelightVisionCameraPoseEstimator implements CameraPoseEstimator{
    // Limelight API documentation: https://docs.limelightvision.io/en/latest/networktables_api.html#apriltag-and-3d-data
    private static final int X_M = 0;
    private static final int Y_M = 1;
    private static final int Z_ROTATION_DEGREES = 5;
    private static final int TOTAL_LATENCY_MS = 6;
    private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry m_hasTargets = m_table.getEntry("tv");
    private final NetworkTableEntry m_botPoseM = m_table.getEntry("botpose_wpiblue");

    @Override
    public void setReferencePose(Pose2d poseM) {
        // do nothing, not a Limelight feature
    }

    private boolean hasTargets() {
        return m_hasTargets.getDouble(0) == 1;
    }

    @Override
    public Optional<CameraPoseEstimate> getLatestEstimate() {
        if (!hasTargets() || !m_botPoseM.exists()) {
            // TODO: vérifier si l'entrée "tv" fonctionne vraiment pour les AprilTags ou s'il ne faut pas détecter l'absence d'estimé par:
            //  - latence élevée > 1 sec OU
            //  - champs X, Y, YAW à 0
            return Optional.empty();
        }

        var poseComponents = m_botPoseM.getDoubleArray(new Double[7]);
        if (poseComponents[X_M] == null || poseComponents[Y_M] == null || poseComponents[TOTAL_LATENCY_MS] == null) {
            return Optional.empty();
        }

        // See https://docs.limelightvision.io/en/latest/apriltags_in_3d.html#using-wpilib-s-pose-estimator
        var timestampS = Timer.getFPGATimestamp() - poseComponents[TOTAL_LATENCY_MS] / 1000;
        var xM = poseComponents[X_M];
        var yM = poseComponents[Y_M];
        var yawDegrees = poseComponents[Z_ROTATION_DEGREES];

        return Optional.of(
            new CameraPoseEstimate(
                new Pose2d(xM, yM, Rotation2d.fromDegrees(yawDegrees)),
                timestampS
            )
        );
    }
}
