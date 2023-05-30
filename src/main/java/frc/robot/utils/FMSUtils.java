package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FMSUtils {
    private static final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("FMSInfo");
    private static final NetworkTableEntry m_isRedAllianceEntry = m_table.getEntry("IsRedAlliance");

    public static Alliance currentAlliance() {
        // return m_isRedAllianceEntry.getBoolean(false) ? Alliance.Red : Alliance.Blue;
        return Alliance.Blue;
    }
}
