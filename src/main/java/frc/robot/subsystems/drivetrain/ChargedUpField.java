package frc.robot.subsystems.drivetrain;

import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PilotShuffleboardLayout;
import frc.robot.utils.FMSUtils;
import frc.robot.utils.NodeType;

/**
 * Donne accès aux coordonnées importantes de certains éléments du terrain.
 * 
 * Document de calcul des coordonnées: https://docs.google.com/presentation/d/1mX9c9Sw2toBV61v2YBbtUmS6ajEZyHKFkF6gAtpS6FU
 */
public class ChargedUpField {
    // Red Node 8 - centrer to bumper, meters: (x = 14.69, y = 4.983)
    // Blue Node 8 - centrer to bumper, meters: (x = 1.85, y = 4.983)
    // Top red game piece, meters: (x = 9.464, y = 4.578)
    // Top blue game piece, meters: (x = 7.076, y = 4.578)
    // Top game piece to node 8: y = 16" = 0.406m

    private static final GridNode[] NODES = {
        GridNode.fromYInches(0, 20.19, NodeType.Cone),
        GridNode.fromYInches(1, 42.19, NodeType.Cube),
        GridNode.fromYInches(2, 64.19, NodeType.Cone),
        GridNode.fromYInches(3, 86.19, NodeType.Cone),
        GridNode.fromYInches(4, 108.19, NodeType.Cube),
        GridNode.fromYInches(5, 130.19, NodeType.Cone),
        GridNode.fromYInches(6, 152.19, NodeType.Cone),
        GridNode.fromYInches(7, 174.19, NodeType.Cube), // 4.424m
        GridNode.fromYInches(8, 196.19, NodeType.Cone)  // 4.983m
    };
    private static final double[] INITIAL_GAME_PIECE_Y_M = {
        Units.inchesToMeters(36.25),
        Units.inchesToMeters(84.25),
        Units.inchesToMeters(132.25),
        Units.inchesToMeters(180.25),
    };
    private static final Rotation2d TOWARDS_BLUE = Rotation2d.fromDegrees(180);
    private static final Rotation2d TOWARDS_RED = Rotation2d.fromDegrees(0);
    private static final Pose2d BLUE_CHARGE_STATION_CENTER_M = new Pose2d(
        Units.inchesToMeters(151.355),
        Units.inchesToMeters(108.015),
        TOWARDS_RED
    );
    private static final Pose2d RED_CHARGE_STATION_CENTER_M = new Pose2d(
        Units.inchesToMeters(499.865),
        Units.inchesToMeters(108.015),
        TOWARDS_BLUE
    );
    private static final double BLUE_GRID_BORDER_X_M = Units.inchesToMeters(54.605); // 1.387m
    private static final double RED_GRID_BORDER_X_M = Units.inchesToMeters(596.615); // 15.154m
    private static final double GRID_TO_INITIAL_PIECE_X_M = Units.inchesToMeters(18 *12 + 8); // 5.69m
    private static final double TEST_GRID_TO_INITIAL_PIECE_X_M = Units.inchesToMeters(10 * 12); // 120", 3.048m (diff = 2.642m)
    private static final double BLUE_INITIAL_PIECES_X_M = BLUE_GRID_BORDER_X_M + GRID_TO_INITIAL_PIECE_X_M; // 278.605", 7.076m
    private static final double RED_INITIAL_PIECES_X_M = RED_GRID_BORDER_X_M - TEST_GRID_TO_INITIAL_PIECE_X_M; // Terrain test 452.615", 11.496m
    // private static final double RED_INITIAL_PIECES_X_M = RED_GRID_BORDER_X_M - GRID_TO_INITIAL_PIECE_X_M; // Vrai terrain 372.615", 9.464m

    
    private static final double BLUE_COMMUNITY_EXIT_X_M = Units.inchesToMeters(235.685);
    private static final double RED_COMMUNITY_EXIT_X_M = Units.inchesToMeters(415.535);
    private static final Rotation2d HALF_TURN = Rotation2d.fromDegrees(180);

    private static final Trajectory GRAB_CUBE_FROM_RED_NODE8 = importTrajectory("GrabCubeFromRedNode8.wpilib.json");
    private static final Trajectory GRAB_CUBE_FROM_BLUE_NODE8 = importTrajectory("GrabCubeFromBlueNode8.wpilib.json");
    private static final Trajectory BRING_CUBE_TO_RED_NODE7 = importTrajectory("BringCubeToRedNode7.wpilib.json");
    private static final Trajectory BRING_CUBE_TO_BLUE_NODE7 = importTrajectory("BringCubeToBlueNode7.wpilib.json");
    private static final Trajectory ENGAGE_FROM_RED_NODE7 = importTrajectory("EngageFromRedNode7.wpilib.json");
    private static final Trajectory ENGAGE_FROM_BLUE_NODE7 = importTrajectory("EngageFromBlueNode7.wpilib.json");
    public static final Trajectory TEST_GRAB_CUBE = importTrajectory("TestGrabFromRedNode8.wpilib.json"); 

    private Optional<Alliance> forceAlliance;
    private final Field2d m_fieldWidget = new Field2d();

    public ChargedUpField() {
        this(Optional.empty());
    }

    public ChargedUpField(Alliance alliance) {
        this(Optional.of(alliance));
    }

    private ChargedUpField(Optional<Alliance> forceAlliance) {
        this.forceAlliance = forceAlliance;

        // Affiche les éléments important du jeu pour le mode autonome        
        for(int i=0; i< NODES.length; i++) {
            final var node = NODES[i];
            m_fieldWidget
                .getObject(String.format("Blue node %d", i))
                .setPose(new Pose2d(BLUE_GRID_BORDER_X_M, node.yM, TOWARDS_RED));
            m_fieldWidget
                .getObject(String.format("Red node %d", i))
                .setPose(new Pose2d(RED_GRID_BORDER_X_M, node.yM, TOWARDS_BLUE));
        }

        for(int i=0; i< INITIAL_GAME_PIECE_Y_M.length; i++) {
            final var yM = INITIAL_GAME_PIECE_Y_M[i];
            m_fieldWidget
                .getObject(String.format("Blue initial piece %d", i))
                .setPose(new Pose2d(BLUE_INITIAL_PIECES_X_M, yM, TOWARDS_BLUE));
            m_fieldWidget
                .getObject(String.format("Red initial piece %d", i))
                .setPose(new Pose2d(RED_INITIAL_PIECES_X_M, yM, TOWARDS_RED));
        }
        SmartDashboard.putData(m_fieldWidget);
        boolean currentAllianceExist = PilotShuffleboardLayout.CMD_LAYOUT.getComponents().stream()
            .anyMatch(comp -> comp.getTitle().equals("CurrentAlliance"));
        if (currentAllianceExist == false) {
            PilotShuffleboardLayout.CMD_LAYOUT.addString("CurrentAlliance", () -> isBlueAlliance() ? "BLUE" : "RED");
        }
        
    }

    private static Trajectory importTrajectory(String fileName) {
        try {
            var trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output").resolve(fileName);
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + fileName, ex.getStackTrace());
            return null;
         }
    }

    public void switchColorAlliance() {
        if (isBlueAlliance()) {
            forceAlliance = Optional.of(Alliance.Red);
        } else {
            forceAlliance = Optional.of(Alliance.Blue);
        }
    }

    /**
     * Met à jour la position du robot dans le widget du terrain.
     */
    public void updateRobotPose(Pose2d robotPoseM) {
        m_fieldWidget.setRobotPose(robotPoseM);
    }

    /**
     * Mets à jour la trajectoire affichée dans le widget du terrain.
     */
    public void displayTrajectory(Trajectory traj) {
        m_fieldWidget.getObject("Trajectory").setTrajectory(traj);
    }

    private boolean isBlueAlliance() {
        return forceAlliance.isPresent() ? forceAlliance.get() == Alliance.Blue : FMSUtils.currentAlliance() == Alliance.Blue;
    }

    public boolean isAllianceColorBlue() {
        return isBlueAlliance();
    }

    private double initialPiecesXM() {
        return isBlueAlliance() ? BLUE_INITIAL_PIECES_X_M : RED_INITIAL_PIECES_X_M;
    }

    private double gridBorderXM() {
        return isBlueAlliance() ? BLUE_GRID_BORDER_X_M : RED_GRID_BORDER_X_M;
    }

    /**
     * The pose of the center of the charge station. Assume facing other alliance.
     */
    public Pose2d chargeStationCenterM() {
        return isBlueAlliance() ? BLUE_CHARGE_STATION_CENTER_M : RED_CHARGE_STATION_CENTER_M;
    }

    public double gridRobotXDirection() {
        return isBlueAlliance() ? -1 : 1;
    }

    /**
     * Get the X coordinate of the community exit point.
     */
    public double exitCommunityXM() {
        return isBlueAlliance() ? BLUE_COMMUNITY_EXIT_X_M : RED_COMMUNITY_EXIT_X_M;
    }

    /**
     * The orientation the robot should be facing when looking at it's alliance grid.
     */
    public Rotation2d towardSelfAlliance() {
        return isBlueAlliance() ? TOWARDS_BLUE : TOWARDS_RED;
    }

    public Rotation2d towardOppositeAlliance() {
        return isBlueAlliance() ? TOWARDS_RED : TOWARDS_BLUE;
    }

    /**
     * Given the Y coordinate of the robot center, determines which grid node is the closest on the Y axis and returns the appropriate pose for the robot to face it perfectly.
     * This is useful to assist the driver align with the grid.
     * @param yM the coordinate of the robot.
     * @return the closest grid node, along with the appropriate pose for the robot to face it.
     */
    public GridNodePose closestGridNodePose(double yM, double robotCenterToFrontBumperM) {
        GridNode closest = NODES[0];
        double closestDistance = Math.abs(yM - closest.yM);

        for(int i=0; i<NODES.length; i++) {
            var distance = Math.abs(yM - NODES[i].yM);
            if (distance < closestDistance) {
                closest = NODES[i];
                closestDistance = distance;
            }
        }

        return new GridNodePose(
            gridNodePose(closest.index, robotCenterToFrontBumperM),
            closest.type);
    }

    /**
     * Given the grid node index, returns the appropriate pose for the robot to face it perfectly.
     * This is useful in autonomous mode to reach the desired node.
     * @param nodeIndex 0 being at the bottom one (near the field border), 8 the top one (towards the sub stations)
     * @param robotCenterToFrontBumperM the distance from the robot center to the front bumper.
     */
    public Pose2d gridNodePose(int nodeIndex, double robotCenterToFrontBumperM) {
        return new Pose2d(
            gridBorderXM() - gridRobotXDirection() * robotCenterToFrontBumperM,
            NODES[nodeIndex].yM,
            towardSelfAlliance()
    );
    }

    /**
     * Given the piece index, returns the pose the robot should have to catch it.
     * This is useful in autonomous mode to reach the desired initial game piece.
     * @param pieceIndex 0 being at the bottom one (near the field border), 3 the top one (towards the sub stations)
     * @param robotCenterToPiece the distance from the robot center to the piece so that the robot is perfectly positionned to move and grab the piece in front of it.
     */
    public Pose2d initialPiecePoseM(int pieceIndex, double robotCenterToPiece) {
        return new Pose2d(
            initialPiecesXM() + gridRobotXDirection() * robotCenterToPiece,
            INITIAL_GAME_PIECE_Y_M[pieceIndex],
            towardSelfAlliance().plus(HALF_TURN)
        );
    }

    public Trajectory grabCubeFromNode8Trajectory() {
        return isBlueAlliance() ? GRAB_CUBE_FROM_BLUE_NODE8 : GRAB_CUBE_FROM_RED_NODE8;
    }

    public Trajectory bringCubeToNode7Trajectory() {
        return isBlueAlliance() ? BRING_CUBE_TO_BLUE_NODE7 : BRING_CUBE_TO_RED_NODE7;
    }

    public Trajectory engageFromNode7Trajectory() {
        return isBlueAlliance() ? ENGAGE_FROM_BLUE_NODE7 : ENGAGE_FROM_RED_NODE7;
    }
}
