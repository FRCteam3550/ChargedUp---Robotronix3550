package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PilotShuffleboardLayout;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousCommands {
    private static DrivetrainSubsystem m_drivetrainSubsystem;
    private static GamepiecePositioning m_gamepiecePositioning;
    private static Claw m_claw;
    public final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public AutonomousCommands(DrivetrainSubsystem drivetrainSubsystem, 
                              GamepiecePositioning gamepiecePositioning, 
                              Claw claw) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_gamepiecePositioning = gamepiecePositioning;
        m_claw = claw;

        autoChooser.addOption("Cone haut, puis sortie noeud 0", dropConeHighAndExitCommunity(0));
        //autoChooser.setDefaultOption("Test trajectory", m_drivetrainSubsystem.testRedGrabCubeTrajectory());
        autoChooser.addOption("Ne Fait Rien", Commands.print("Ne Fait Rien"));  
        autoChooser.addOption("Cone haut, puis sortie noeud 8", dropConeHighAndExitCommunity(8));
        autoChooser.setDefaultOption("Cone haut, puis equilibre noeud 3", dropConeHighAndEngageChargingStationFromNode3());
        autoChooser.addOption("Cone haut", dropConeHigh());
        autoChooser.addOption("Sortie noeud 0", m_drivetrainSubsystem.exitCommunityFromNode(0));
        autoChooser.addOption("Sortie noeud 8", m_drivetrainSubsystem.exitCommunityFromNode(8));
        autoChooser.addOption("Equilibre noeud 3", m_drivetrainSubsystem.engageChargingStationFromNodeAndBalance());
        autoChooser.addOption("Sortie, puis équilibre noeud 3", m_drivetrainSubsystem.exitCommunityAndEngagedOption2());
        autoChooser.addOption("Cone haut, sortie, puis équilibre noeud 3", dropConeHighExitAndBalanceFromNode3Op2());

        PilotShuffleboardLayout.PREP_LAYOUT.add("Choix auto", autoChooser);
    }

    public Command getSelected() {
        return autoChooser.getSelected();
    }

    public Command dropConeHigh() {
        return m_claw.takeCone()
            .andThen(m_gamepiecePositioning.positionGamePieceHighAuto())
            .withName("dropConeHigh");
    }    

    public Command dropConeMedium() {
        return m_claw.takeCone()
            .andThen(m_gamepiecePositioning.positionGamePieceMedium())
            .withName("dropConeMedium");
    }   

    public Command dropConeLow() {
        return m_claw.takeCone()
            .andThen(m_gamepiecePositioning.positionGamePieceLow())
            .withName("dropConeLow");
    }   

    public Command dropCubeHigh() {
        return m_claw.takeCube()
            .andThen(m_gamepiecePositioning.positionGamePieceHighAuto())
            .withName("dropCubeHigh");
    }

    public Command dropCubeMedium() {
        return m_claw.takeCube()
            .andThen(m_gamepiecePositioning.positionGamePieceMedium())
            .withName("dropCubeMedium");
    }  

    public Command dropCubeLow() {
        return m_claw.takeCube()
            .andThen(m_gamepiecePositioning.positionGamePieceLow())
            .withName("dropCubeLow");
    }

    public Command dropConeHighAndExitCommunity(int fromNodeIndex) {
        return dropConeHigh()
            .andThen(m_gamepiecePositioning.goToIdlePosition())
            .andThen(m_drivetrainSubsystem.exitCommunityFromNode(fromNodeIndex))
            .andThen(m_drivetrainSubsystem.halfTurn())
            .withName("dropConeHighAndExitCommunity");     
    }

    public Command dropConeHighAndEngageChargingStationFromNode3() {
        return dropConeHigh()
            .andThen(
                m_drivetrainSubsystem.engageChargingStationFromNodeAndBalance()
                    .alongWith(m_gamepiecePositioning.goToIdlePosition())
            )
            .withName("dropConeHighAndEngageChargingStationFromNode3");
    }

    public Command dropConeHighExitAndBalanceFromNode3Op2() {
        return dropConeHigh()
        .andThen(
            m_drivetrainSubsystem.exitCommunityAndEngagedOption2()
                .alongWith(m_gamepiecePositioning.goToIdlePosition())
        )
        .withName("dropConeHighExitAndBalanceFromNode3Op2");
    }

    public Command trajectoryFollowing2PiecesAutoWithHalfTurn() {
        return m_drivetrainSubsystem.trajectory2PiecesAutoWithHalfTurn()
        .andThen(m_gamepiecePositioning.grabCubeFloor())
        .withName("trajectoryFollowing2PiecesAutoWithHalfTurn");
    }

    public Command grabCubeFromNode8() {
        return m_drivetrainSubsystem.grabCubeFromNode8()
            .alongWith(m_gamepiecePositioning.grabCubeFloor())
            .withName("grabCubeFromNode8");
    }

    public Command grabCubeFromNode8AndBringItBack() {
        return m_drivetrainSubsystem.grabCubeFromNode8()
            .alongWith(m_gamepiecePositioning.grabCubeFloor())
            .andThen(m_drivetrainSubsystem.bringCubeToNode7())
            .andThen(m_gamepiecePositioning.positionGamePieceLow())
            .withName("grabCubeFromNode8AndBringItBack");
    }
}
