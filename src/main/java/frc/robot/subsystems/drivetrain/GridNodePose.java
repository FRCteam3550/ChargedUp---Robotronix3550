package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.NodeType;

public class GridNodePose {
    public final Pose2d poseM;
    public final NodeType type;

    public GridNodePose(Pose2d poseM, NodeType type) {
        this.poseM = poseM;
        this.type = type;
    }
}
