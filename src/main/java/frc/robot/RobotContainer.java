package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutonomousCommands;
import frc.robot.commands.GamepiecePositioning;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.ChargedUpField;

public class RobotContainer {
    private enum POV {
        UP(0),
        RIGHT(90),
        DOWN(180),
        LEFT(270);

        private final int degrees;

        POV(int degres) {
          this.degrees = degres;
        }
    }

    private final XboxController m_pilotGamepad = new XboxController(0);
    private final JoystickButton m_aPilotButton = new JoystickButton(m_pilotGamepad, XboxController.Button.kA.value);
    private final JoystickButton m_bPilotButton = new JoystickButton(m_pilotGamepad, XboxController.Button.kB.value);
    private final JoystickButton m_xPilotButton = new JoystickButton(m_pilotGamepad, XboxController.Button.kX.value);
    private final JoystickButton m_yPilotButton = new JoystickButton(m_pilotGamepad, XboxController.Button.kY.value);
    private final JoystickButton m_tlPilotButton = new JoystickButton(m_pilotGamepad, XboxController.Button.kLeftBumper.value);
    private final JoystickButton m_trPilotButton = new JoystickButton(m_pilotGamepad, XboxController.Button.kRightBumper.value);
    //private final Trigger m_povUpButton = povButton(m_pilotGamepad, POV.UP); 
    // private final Trigger m_povLeftButton = povButton(m_pilotGamepad, POV.LEFT); 
    private final Trigger m_povDownButton = povButton(m_pilotGamepad, POV.DOWN); 
    private final Trigger m_povUpButton = povButton(m_pilotGamepad, POV.UP); 
    private final Trigger m_povRightButton = povButton(m_pilotGamepad, POV.RIGHT); 
    private final JoystickButton m_startPilotButton = new JoystickButton(m_pilotGamepad, XboxController.Button.kStart.value);

    private final XboxController m_coPilotGamepad = new XboxController(1);
    private final JoystickButton m_yCoPilotButton = new JoystickButton(m_coPilotGamepad, XboxController.Button.kY.value);
    private final JoystickButton m_aCoPilotButton = new JoystickButton(m_coPilotGamepad, XboxController.Button.kA.value);
    private final JoystickButton m_xCoPilotButton = new JoystickButton(m_coPilotGamepad, XboxController.Button.kX.value);
    private final JoystickButton m_bCoPilotButton = new JoystickButton(m_coPilotGamepad, XboxController.Button.kB.value);
    private final JoystickButton m_backCoPilotButton = new JoystickButton(m_coPilotGamepad, XboxController.Button.kBack.value);
    private final JoystickButton m_trCoPilotButton = new JoystickButton(m_coPilotGamepad, XboxController.Button.kRightBumper.value);
    private final JoystickButton m_tlCoPilotButton = new JoystickButton(m_coPilotGamepad, XboxController.Button.kLeftBumper.value);
    private final JoystickButton m_startCoPilotButton = new JoystickButton(m_coPilotGamepad, XboxController.Button.kStart.value);
    private final Trigger m_povUpCoPilotButton = povButton(m_coPilotGamepad, POV.UP);

    //    private final JoystickButton m_testCoPilot = new JoystickButton(m_coPilotGamepad, XboxController.Button.kB.value);

    //private final Trigger m_povDownCoPilotButton = povButton(m_coPilotGamepad, POV.DOWN);
//    private final JoystickButton m_testCoPilot = new JoystickButton(m_coPilotGamepad, XboxController.Button.kB.value);
    //private final Trigger m_povDownCoPilotButton = povButton(m_coPilotGamepad, POV.DOWN);

    private final XboxController m_testPilotGamepad = new XboxController(2);
    private final JoystickButton m_cameraUpTest = new JoystickButton(m_testPilotGamepad, XboxController.Button.kY.value);
    private final JoystickButton m_cameraDownTest = new JoystickButton(m_testPilotGamepad, XboxController.Button.kA.value);
    private final JoystickButton m_goToTwentyCmElevator = new JoystickButton(m_testPilotGamepad, XboxController.Button.kB.value);
    private final JoystickButton m_goToFourtyCmElevator = new JoystickButton(m_testPilotGamepad, XboxController.Button.kX.value);
    private final JoystickButton m_PIDTester = new JoystickButton(m_testPilotGamepad, XboxController.Button.kBack.value);
    

    private final DigitalInput m_brakeModeSwitch = new DigitalInput(Constants.BRAKE_MODE_LIMITSWITCH_ID);
    private final Trigger m_brakeModeButton = new Trigger(m_brakeModeSwitch::get);
    
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(m_pilotGamepad);
    private final Elevator m_elevator = new Elevator();
    private final Extensor m_extensor = new Extensor();
    private final Pivot m_pivot = new Pivot();
    private final Claw m_claw = new Claw();
    private final Cameras m_cameras = new Cameras();
    private final Leds m_leds = new Leds();

    private final GamepiecePositioning m_gamepiecePositioning = new GamepiecePositioning(m_claw, m_elevator, m_extensor, m_pivot, m_cameras);
    private final AutonomousCommands m_autonomusCommands = new AutonomousCommands(m_drivetrainSubsystem, m_gamepiecePositioning, m_claw);

    public RobotContainer() {
        PilotShuffleboardLayout.CMD_LAYOUT.add(CommandScheduler.getInstance());
        configureButtonBindings();
    }

    private Trigger povButton(XboxController gamepad, POV button) {
        return new Trigger(() -> gamepad.getPOV(0) == button.degrees);
    }

    private void configureButtonBindings() {
        m_aPilotButton.onTrue(m_gamepiecePositioning.grabCubeFloor());
        m_bPilotButton.onTrue(m_gamepiecePositioning.grabStandingConeFloor());
        m_xPilotButton.onTrue(m_gamepiecePositioning.grabCubeSubStation());
        m_yPilotButton.onTrue(m_gamepiecePositioning.grabConeSubStation());
        m_povDownButton.onTrue(m_gamepiecePositioning.grabFallenConeFloor());
        m_povUpButton.onTrue(m_cameras.cameraToggle());
        m_tlPilotButton.onTrue(m_drivetrainSubsystem.switchToPreciseMode());
        m_trPilotButton.onTrue(m_drivetrainSubsystem.switchToFastMode());
        m_startPilotButton.onTrue(m_gamepiecePositioning.cancelAll());
        m_povRightButton.onTrue(m_drivetrainSubsystem.switchAlliance());

        m_aCoPilotButton.onTrue(m_gamepiecePositioning.positionGamePieceLow());
        m_xCoPilotButton.onTrue(m_gamepiecePositioning.prepareGamePieceMedium());
        m_xCoPilotButton.onFalse(m_gamepiecePositioning.releaseGamePieceMedium());
        m_yCoPilotButton.onTrue(m_gamepiecePositioning.prepareGamePieceHigh());
        m_yCoPilotButton.onFalse(m_gamepiecePositioning.releaseGamePieceHigh());
        m_backCoPilotButton.onTrue(m_gamepiecePositioning.switchBrakeAndCoast());
        m_brakeModeButton.onTrue(m_gamepiecePositioning.switchBrakeAndCoast());
        
        m_povUpCoPilotButton.onTrue(m_drivetrainSubsystem.trajectory2PiecesAutoWithHalfTurn());
        m_bCoPilotButton.onTrue(m_drivetrainSubsystem.trajectory2PiecesAuto());

        m_trCoPilotButton.onTrue(m_leds.wantCone());
        m_tlCoPilotButton.onTrue(m_leds.wantCube());
        m_startCoPilotButton.onTrue(m_leds.stopAsking());

        m_cameraUpTest.onTrue(m_cameras.switchCameraUp());
        m_cameraDownTest.onTrue(m_cameras.switchCameraDown());

        m_goToTwentyCmElevator.onTrue(m_elevator.goToTwentyCM());
        m_goToFourtyCmElevator.onTrue(m_elevator.goToFourthCM());

        m_PIDTester.onTrue(m_drivetrainSubsystem.testPIDSpeed());
    }

    public Command getAutonomousCommand() {
        return m_autonomusCommands.getSelected();
    }
}
