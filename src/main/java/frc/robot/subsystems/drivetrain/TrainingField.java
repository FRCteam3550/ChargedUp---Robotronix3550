package frc.robot.subsystems.drivetrain;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Models our Robotronix training field with our 2 tags on the far wall.
 */
public class TrainingField {
    public static final AprilTag TAG0 = new AprilTag(0, new Pose3d(1.988, 4.54, 0.803, new Rotation3d(0, 0, Math.toRadians(-90))));
    public static final AprilTag TAG1 = new AprilTag(1, new Pose3d(0.962, 4.54, 0.803, new Rotation3d(0, 0, Math.toRadians(-90))));
    public static final AprilTagFieldLayout LAYOUT = new AprilTagFieldLayout(List.of(TAG0, TAG1), 4.54, 4.63); 
}
