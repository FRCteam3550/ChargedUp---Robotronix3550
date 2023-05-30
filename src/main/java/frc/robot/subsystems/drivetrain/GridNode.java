package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.NodeType;

public class GridNode {
    public final int index;
    public final double yM;
    public final NodeType type;

    public GridNode(int index, double yM, NodeType type) {
        this.index = index;
        this.yM = yM;
        this.type = type;
    }

    public static GridNode fromYInches(int index, double yInches, NodeType type) {
        return new GridNode(index, Units.inchesToMeters(yInches), type);
    }
}
