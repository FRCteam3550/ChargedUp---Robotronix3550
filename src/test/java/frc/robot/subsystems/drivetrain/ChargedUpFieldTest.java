package frc.robot.subsystems.drivetrain;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.NodeType;

import static org.junit.jupiter.api.Assertions.*;

public class ChargedUpFieldTest {
    private static double CENTER_TO_FRONT_BUMPER_M = 0.46;
    private static double CENTER_TO_GAME_PIECE_M = 0.8;
    private static double BLUE_GRID_BORDER_X_M = Units.inchesToMeters(54.605);
    private static double RED_GRID_BORDER_X_M = Units.inchesToMeters(596.615);
    private static double BLUE_INITIAL_PIECES_X_M = Units.inchesToMeters(278.605);
    private static double RED_INITIAL_PIECES_X_M = Units.inchesToMeters(472.615);

    @Test
    void whenBlueClosestGridNodePose() {
        var field = new ChargedUpField(Alliance.Blue);

        // 5th node is at Y = 130.19" and is a cone node
        var near5thNodeYM = Units.inchesToMeters(120);
        var result = field.closestGridNodePose(near5thNodeYM, CENTER_TO_FRONT_BUMPER_M);

        assertEquals(BLUE_GRID_BORDER_X_M + CENTER_TO_FRONT_BUMPER_M, result.poseM.getX());
        assertEquals(Units.inchesToMeters(130.19), result.poseM.getY());
        assertEquals(Rotation2d.fromDegrees(180), result.poseM.getRotation());
        assertEquals(NodeType.Cone, result.type);
    }    

    @Test
    void whenRedClosestGridNodePose() {
        var field = new ChargedUpField(Alliance.Red);

        // 4th node is at Y = 108.19" and is a cube node
        var near4thNodeYM = Units.inchesToMeters(118);
        var result = field.closestGridNodePose(near4thNodeYM, CENTER_TO_FRONT_BUMPER_M);

        assertEquals(RED_GRID_BORDER_X_M - CENTER_TO_FRONT_BUMPER_M, result.poseM.getX());
        assertEquals(Units.inchesToMeters(108.19), result.poseM.getY());
        assertEquals(Rotation2d.fromDegrees(0), result.poseM.getRotation());
        assertEquals(NodeType.Cube, result.type);
    }

    @Test
    void whenBlueGridNodePose() {
        var field = new ChargedUpField(Alliance.Blue);

        // 5th node is at Y = 130.19" and is a cone node
        var result = field.gridNodePose(8, CENTER_TO_FRONT_BUMPER_M);

        assertEquals(BLUE_GRID_BORDER_X_M + CENTER_TO_FRONT_BUMPER_M, result.getX());
        assertEquals(Units.inchesToMeters(196.19), result.getY());
        assertEquals(Rotation2d.fromDegrees(180), result.getRotation());
    }    

    @Test
    void whenRedGridNodePose() {
        var field = new ChargedUpField(Alliance.Red);

        // 4th node is at Y = 108.19" and is a cube node
        var result = field.gridNodePose(8, CENTER_TO_FRONT_BUMPER_M);

        assertEquals(RED_GRID_BORDER_X_M - CENTER_TO_FRONT_BUMPER_M, result.getX());
        assertEquals(Units.inchesToMeters(196.19), result.getY());
        assertEquals(Rotation2d.fromDegrees(0), result.getRotation());
    }

    // @Test
    // void whenBlueInitialGamePiecePose() {
    //     var field = new ChargedUpField(Alliance.Blue);

    //     var result = field.initialPiecePoseM(3, CENTER_TO_GAME_PIECE_M);

    //     assertEquals(BLUE_INITIAL_PIECES_X_M - CENTER_TO_GAME_PIECE_M, result.getX());
    //     assertEquals(Units.inchesToMeters(147.25), result.getY());
    //     assertEquals(Rotation2d.fromDegrees(0), result.getRotation());
    // }    

    // @Test
    // void whenRedInitialGamePiecePose() {
    //     var field = new ChargedUpField(Alliance.Red);

    //     var result = field.initialPiecePoseM(1, CENTER_TO_GAME_PIECE_M);

    //     assertEquals(RED_INITIAL_PIECES_X_M + CENTER_TO_GAME_PIECE_M, result.getX());
    //     assertEquals(Units.inchesToMeters(51.25), result.getY());
    //     assertEquals(Rotation2d.fromDegrees(180), result.getRotation());
    // }

    @Test
    void whenBlueChargeStationCenter() {
        var field = new ChargedUpField(Alliance.Blue);

        assertEquals(Units.inchesToMeters(151.355), field.chargeStationCenterM().getX());
    }     

    @Test
    void whenRedChargeStationCenter() {
        var field = new ChargedUpField(Alliance.Red);

        assertEquals(Units.inchesToMeters(499.865), field.chargeStationCenterM().getX());
    }     
}
