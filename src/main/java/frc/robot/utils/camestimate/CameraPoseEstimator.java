package frc.robot.utils.camestimate;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;

public interface CameraPoseEstimator {
    public class CameraPoseEstimate {
        public final double timestampS;
        public final Pose2d poseM;

        public CameraPoseEstimate(Pose2d poseM, double timestampS) {
            this.poseM = poseM;
            this.timestampS = timestampS;
        }
    }

    /**
     * Some camera pose estimators, like PhotonVision, can use a reference pose.
     */
    void setReferencePose(Pose2d poseM);

    /**
     * Get the current estimate of the Robot's pose in field reference, if any.
     */
    Optional<CameraPoseEstimate> getLatestEstimate();
}