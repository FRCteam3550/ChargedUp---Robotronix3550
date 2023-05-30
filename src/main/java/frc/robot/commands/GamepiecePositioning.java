package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PilotShuffleboardLayout;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Extensor;
import frc.robot.subsystems.Pivot;

public class GamepiecePositioning {
    private final Claw m_claw;
    private final Elevator m_elevator;
    private final Extensor m_extensor;
    private final Pivot m_pivot;
    private final Cameras m_camera;

    private boolean brake = true;
    private static enum CurrentPhase {
        Working,
        Finished
    }
    private CurrentPhase m_piecePositioningPhase = CurrentPhase.Finished;
    
    public GamepiecePositioning(Claw claw, Elevator elevator, Extensor extensor, Pivot pivot, Cameras camera) {
        this.m_claw = claw;
        this.m_elevator = elevator;
        this.m_extensor = extensor;
        this.m_pivot = pivot;
        this.m_camera = camera;

        PilotShuffleboardLayout.PREP_LAYOUT.add(prepareForCone());
        PilotShuffleboardLayout.CMD_LAYOUT.addString("Mode repos", () -> brake ? "FREIN" : "LIBRE");
    }

    public Command goToIdlePosition() {
        return m_elevator.goToBottom()
        .alongWith(m_extensor.goInside())
        .alongWith(m_pivot.goToRetractedAngle())
        .withName("goToIdlePosition");
    }

    public Command goToIdlePositionConeSubstation() {
        return m_elevator.goToBottom()
        .alongWith(m_extensor.goInside())
        .alongWith(m_pivot.goToRetractedAngleSubstationCone())
        .andThen(m_pivot.goToRetractedAngle())
        .withName("goToIdlePositionConeSubstation");

    }

    public Command positionGamePieceHigh() {
        var whenCone = m_elevator.goToTopGridRowCone()
        .alongWith(m_extensor.goToTopGridRowCone())
        .alongWith(m_pivot.goToUp())
        .andThen(m_pivot.goToHighRowCone());

        var whenCube = m_elevator.goToTopGridRowCube()
        .alongWith(m_extensor.goToTopGridRowCube())
        .alongWith(m_pivot.goToHighRowCube());

        return Commands.either(whenCone, whenCube, () -> m_claw.isConeGrabbed())
        .andThen(m_claw.reject())
        .andThen(goToIdlePosition())
        .withName("positionGamePieceHigh");
    }

    public Command positionGamePieceHighAuto() {
        var whenCone = m_elevator.goToTopGridRowCone()
        .alongWith(m_extensor.goToTopGridRowCone())
        .alongWith(m_pivot.goToUp())
        .alongWith(m_claw.takeCone())
        .andThen(m_pivot.goToHighRowCone());

        var whenCube = m_elevator.goToTopGridRowCube()
        .alongWith(m_extensor.goToTopGridRowCube())
        .alongWith(m_pivot.goToHighRowCube())
        .alongWith(m_claw.takeCube());

        return Commands.either(whenCone, whenCube, () -> m_claw.isConeGrabbed())
        .andThen(m_claw.reject())
        .withName("positionGamePieceHighAuto");
    }

    public Command prepareGamePieceHigh() {
        var whenCone = m_elevator.goToTopGridRowCone()
        .alongWith(m_extensor.goToTopGridRowCone())
        .alongWith(m_pivot.goToUp())
        .andThen(m_pivot.goToHighRowCone());

        var whenCube = m_elevator.goToTopGridRowCube()
        .alongWith(m_extensor.goToTopGridRowCube())
        .alongWith(m_pivot.goToHighRowCube());

        return Commands.either(whenCone, whenCube, () -> m_claw.isConeGrabbed())
        .beforeStarting(() -> m_piecePositioningPhase = CurrentPhase.Working)
        .andThen(() -> m_piecePositioningPhase = CurrentPhase.Finished)
        .withName("prepareGamePieceHigh");
    }

    public Command releaseGamePieceHigh() {
        var whenFinished = m_claw.reject()
        .andThen(goToIdlePosition());

        var whenCancelled = goToIdlePosition();

        return Commands.either(whenFinished, whenCancelled, () -> m_piecePositioningPhase == CurrentPhase.Finished)
        .withName("releaseGamePieceHigh");
    }

    public Command positionGamePieceMedium() {
        var whenCone = m_elevator.goToMediumGridRowCone()
        .alongWith(m_extensor.goToMediumGridRowCone())
        .alongWith(m_pivot.goToUp())
        .andThen(m_pivot.goToMiddleRowCone());
        
        var whenCube = m_elevator.goToMediumGridRowCube()
        .alongWith(m_extensor.goToMediumGridRowCube())
        .alongWith(m_pivot.goToMiddleRowCube());

        return Commands.either(whenCone, whenCube, () -> m_claw.isConeGrabbed())
        .andThen(m_claw.reject())
        .andThen(goToIdlePosition())
        .withName("positionGamePieceMedium");
    }

    public Command prepareGamePieceMedium() {
        var whenCone = m_elevator.goToMediumGridRowCone()
        .alongWith(m_extensor.goToMediumGridRowCone())
        .alongWith(m_pivot.goToUp())
        .andThen(m_pivot.goToMiddleRowCone());
        
        var whenCube = m_elevator.goToMediumGridRowCube()
        .alongWith(m_extensor.goToMediumGridRowCube())
        .alongWith(m_pivot.goToMiddleRowCube());

        return Commands.either(whenCone, whenCube, () -> m_claw.isConeGrabbed())
        .beforeStarting(() -> m_piecePositioningPhase = CurrentPhase.Working)
        .andThen(() -> m_piecePositioningPhase = CurrentPhase.Finished)
            .withName("prepareGamePieceMedium");
    }
    
    public Command releaseGamePieceMedium() {
        var whenFinished = m_claw.reject()
        .andThen(goToIdlePosition());
        
        var whenCancelled = goToIdlePosition();

        return Commands.either(whenFinished, whenCancelled, () -> m_piecePositioningPhase == CurrentPhase.Finished)
        .withName("releaseGamePieceMedium");
    }

    public Command positionGamePieceLow() {
        return m_camera.switchCameraDown()
        .alongWith(m_extensor.goInside())
        .alongWith(m_elevator.goToBottom())
        .alongWith(m_pivot.goToHorizontal())
        .andThen(m_claw.reject())
        .andThen(goToIdlePosition())
        .andThen(m_camera.switchCameraUp())
        .withName("positionGamePieceLow");
    }

    public Command grabStandingConeFloor() {
        return m_camera.switchCameraDown()
        .alongWith(m_elevator.goToBottom())
        .alongWith(m_extensor.goInside())
        .alongWith(m_pivot.goToFloorCone())
        .andThen(m_claw.takeCone())
        .andThen(goToIdlePosition())
        .andThen(m_camera.switchCameraUp())
        .withName("grabStandingConeFloor");
    }
    public Command grabFallenConeFloor() {
        return m_camera.switchCameraDown()
        .alongWith(m_elevator.goToFallenCone())
        .alongWith(m_extensor.goToFallenCone())
        .alongWith(m_pivot.goToHorizontal())
        .andThen(m_pivot.goToDownAngle().alongWith(m_claw.open()))
        .andThen(m_claw.takeFallenCone())
        .andThen(m_pivot.goToHorizontal())
        .andThen(goToIdlePosition())
        .andThen(m_camera.switchCameraUp())
        .withName("grabFallenConeFloor");
    }
    public Command grabCubeFloor() {
        return m_camera.switchCameraDown()
        .alongWith(m_elevator.goToBottom())
        .alongWith(m_extensor.goInside())
        .alongWith(m_pivot.goToFloorCube())
        .andThen(m_claw.takeCube())
        .andThen(goToIdlePosition())
        .andThen(m_camera.switchCameraUp())
        .withName("grabCubeFloor");
    }

    public Command grabConeSubStation() {
        return m_camera.switchCameraUp()
        .alongWith(m_elevator.goToLoadingZoneTablet())
        .alongWith(m_extensor.goToLoadingZoneTablet())
        .andThen(m_pivot.goToSubstationCone())
        .andThen(m_claw.takeCone())
        .andThen(goToIdlePositionConeSubstation())
        .withName("grabConeSubStation");
    }

    public Command grabCubeSubStation() {
        return m_camera.switchCameraUp()
        .alongWith(m_elevator.goToLoadingZoneTablet())
        .alongWith(m_extensor.goToLoadingZoneTablet())
        .andThen(m_pivot.goToSubstationCube())
        .andThen(m_claw.takeCube())
        .andThen(goToIdlePosition())
        .withName("grabCubeSubStation");
    }

    public CommandBase prepareForCone() {
        return m_claw.close()
            .andThen(switchToBrake())
            .withName("Prep Cone");
    }

    public Command switchBrakeAndCoast() {
        var switchCoast = m_elevator.switchCoast()
            .alongWith(m_extensor.switchCoast())
            .alongWith(m_pivot.switchCoast())
            .alongWith(m_claw.switchCoast())
            .andThen(() -> brake = false);

        return Commands.either(switchCoast, switchToBrake(), () -> brake)
            .ignoringDisable(true)
            .withName("switchBrakeAndCoast");
    }

    private Command switchToBrake() {
        return m_elevator.switchBrake()
            .alongWith(m_extensor.switchBrake())
            .alongWith(m_pivot.switchBrake())
            .alongWith(m_claw.switchBrake())
            .andThen(() -> brake = true)
            .withName("switchToBrake")
            .ignoringDisable(true);
    }

    public Command cancelAll() {
        return m_pivot.cancelPid()
            .alongWith(m_claw.cancelInsertion());
    }
}
