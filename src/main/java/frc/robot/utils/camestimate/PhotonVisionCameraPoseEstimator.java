package frc.robot.utils.camestimate;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonVisionCameraPoseEstimator implements CameraPoseEstimator, Sendable {
    private static final Transform3d CAMERA_POS = new Transform3d(
        new Translation3d(0.334, -0.279, 1.244), 
        new Rotation3d(Math.toRadians(-90.0), Math.toRadians(-29.36), Math.toRadians(-2.0))
    );
    private static final String CAMERA_NAME = "Microsoft_LifeCam_HD-3000";
    private final PhotonPoseEstimator m_cameraPoseEstimator;
    private double m_cameraLatencyAdjustmentS = -0.035;

    public PhotonVisionCameraPoseEstimator(AprilTagFieldLayout fieldLayout) {
        m_cameraPoseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            new PhotonCamera(CAMERA_NAME),
            CAMERA_POS
        );

        SendableRegistry.add(this, "PhotonVision Pose Estimator");
        SmartDashboard.putData(this);
    }

    @Override
    public void setReferencePose(Pose2d poseM) {
        m_cameraPoseEstimator.setReferencePose(poseM);
    }

    @Override
    public Optional<CameraPoseEstimate> getLatestEstimate() {
        return m_cameraPoseEstimator.update().map((photonEstimate) -> 
            new CameraPoseEstimate(
                photonEstimate.estimatedPose.toPose2d(),
                photonEstimate.timestampSeconds + m_cameraLatencyAdjustmentS
            )
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "camera latency",
            () -> m_cameraLatencyAdjustmentS,
            (value) -> m_cameraLatencyAdjustmentS = value
        );
    }
}
