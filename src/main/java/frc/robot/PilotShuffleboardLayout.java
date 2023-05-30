package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PilotShuffleboardLayout {
    public static final ShuffleboardTab PILOT_TAB = Shuffleboard.getTab("Pilotage");
    public static final ShuffleboardLayout PREP_LAYOUT = PILOT_TAB.getLayout("Prep", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(0, 0);
    public static final ShuffleboardLayout MECHANISMS_LAYOUT = PILOT_TAB.getLayout("Meca", BuiltInLayouts.kList)
        .withSize(1, 4)
        .withPosition(2, 0);
    public static final ShuffleboardLayout ODO_LAYOUT = PILOT_TAB.getLayout("Odo", BuiltInLayouts.kList)
        .withSize(1, 4)
        .withPosition(3, 0);
    public static final ShuffleboardLayout LS_LAYOUT = PILOT_TAB.getLayout("LS", BuiltInLayouts.kList)
        .withSize(1, 5)
        .withPosition(4, 0);
    public static final ShuffleboardLayout CMD_LAYOUT = PILOT_TAB.getLayout("Commandes", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(5, 0);
}
