package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class Pose {
    public static Pose2d withNewX(Pose2d base, double newX) {
        return new Pose2d(newX, base.getY(), base.getRotation());
    }

    public static Pose2d withNewX(Pose2d base, Pose2d newXSource) {
        return new Pose2d(newXSource.getX(), base.getY(), base.getRotation());
    }
}
