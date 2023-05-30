package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PilotShuffleboardLayout;
import frc.robot.utils.GamePiece;

public class Claw extends SubsystemBase {
    private static final double OPEN_CLOSE_TIMEOUT_S = 0.1;

    private static enum InsertionPhase {
        WaitForGrab,
        SecureObject,
        Finished 
    }

    private class insertObjectCommand extends CommandBase {
        private InsertionPhase m_phase = InsertionPhase.WaitForGrab;
        private Timer m_securizationTimer = new Timer();

        public insertObjectCommand() {
            addRequirements(Claw.this);
        }

        @Override
        public void initialize() {
            m_cancelInsertion = false;
            m_phase = InsertionPhase.WaitForGrab;
            turnOffCatchLed();

            m_motor.set(TalonSRXControlMode.PercentOutput, INSERT_OBJECT_SPEED_PCT);
        }

        @Override
        public void execute() {
            switch(m_phase) {
                case WaitForGrab :
                    if(m_cancelInsertion) {   
                        m_phase = InsertionPhase.Finished;
                    } else if(isLimitSwitchActivated()) {
                        m_phase = InsertionPhase.SecureObject;
                        m_securizationTimer.reset();
                        m_securizationTimer.start();
                        turnOnCatchLed();
                    }
                    break;
                case SecureObject :
                    if (m_cancelInsertion) {
                        m_phase = InsertionPhase.Finished;
                    } else if(m_securizationTimer.get() > SECURING_TIMEOUT_S) {
                        m_phase = InsertionPhase.Finished;
                    }
                    break;
                case Finished :
                    break;
            }
        }

        @Override
        public void end(boolean interrupted) {
            stopMotor();
        }

        @Override
        public boolean isFinished() {
            return m_phase == InsertionPhase.Finished;
        }
    }

    private static final double REJECT_OBJECT_SPEED_PCT = -0.3;
    private static final double INSERT_OBJECT_SPEED_PCT = 0.6;

    private static final double REJECT_TIMEOUT_S = 0.5;
    private static final double SECURING_TIMEOUT_S = 0.4;
    private static final double FALLEN_CONE_TIMEOUT_S = 0.5;

    private final TalonSRX m_motor = new TalonSRX(Constants.Claw.MOTOR_ID);

    private final DigitalInput m_limitSwitch = new DigitalInput(Constants.Claw.LIMIT_SWITCH_ID); 

    private final DoubleSolenoid m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Claw.FORWARD_PCM_CHANNEL, Constants.Claw.REVERSE_PCM_CHANNEL);
    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    private final Solenoid m_led = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Claw.CATCH_PCM_LED_CHANNEL);
    
    private GamePiece m_gamePiece = GamePiece.Cone;
    private boolean m_cancelInsertion = false;

    public Claw() {

        SmartDashboard.putData(this);

        stopMotor();
        m_compressor.enableDigital();

        PilotShuffleboardLayout.MECHANISMS_LAYOUT.addString("Pince", () -> isOpen() ? "OUVERT" : "FERME");
        PilotShuffleboardLayout.LS_LAYOUT.addBoolean("Contact piece", this::isLimitSwitchActivated);

        m_motor.setNeutralMode(NeutralMode.Brake);
    }

    public Command switchBrake() {
        return Commands.runOnce(() -> {
            m_motor.setNeutralMode(NeutralMode.Brake);
        }).withName("switchBrake");
    }

    public Command switchCoast() {
        return Commands.runOnce(() -> {
            m_motor.setNeutralMode(NeutralMode.Coast);
        }).withName("switchCoast");
    }

    public boolean isConeGrabbed() {
        return m_gamePiece == GamePiece.Cone;
    }

    private boolean isOpen() {
        return m_solenoid.get() == Value.kReverse;
    }

    private void openSolenoid() {
        m_solenoid.set(Value.kReverse);
    }

    private void closeSolenoid() {
        m_solenoid.set(Value.kForward);
    }

    public CommandBase open() {
        return run(this::openSolenoid).withTimeout(OPEN_CLOSE_TIMEOUT_S);
    }

    
    public CommandBase close() {
        return run(this::closeSolenoid).withTimeout(0.1);
    }

    public Command takeCube() {
        return run(() -> {
            openSolenoid();
            m_gamePiece = GamePiece.Cube;
        })
        .withTimeout(OPEN_CLOSE_TIMEOUT_S)
        .andThen(insertObject())
        .withName("takeCube");
    }

    public Command takeFallenCone() {
        return run(() -> {
            openSolenoid();
            m_gamePiece = GamePiece.Cone;
        })
        .withTimeout(FALLEN_CONE_TIMEOUT_S)
        .andThen(close())
        .andThen(insertObject())
        .withName("takeFallenCone");
    }

    public Command takeCone() {
        return run(() -> {
            closeSolenoid();
            m_gamePiece = GamePiece.Cone;
        })
        .withTimeout(OPEN_CLOSE_TIMEOUT_S)
        .andThen(insertObject())
        .withName("takeCone");
    }

    private boolean isLimitSwitchActivated() {
        return !m_limitSwitch.get();
    }

    private Command insertObject() {
        return new insertObjectCommand();
    }

    public Command cancelInsertion() {
        return Commands.runOnce(() -> m_cancelInsertion = true)
            .withName("cancelInsertion");
    }

    public Command reject() {
        return Commands.either(rejectCone(), rejectCube(), () -> m_gamePiece == GamePiece.Cone)
            .withName("reject");   
    }

    private Command rejectCube() {
        return run(() -> m_motor.set(TalonSRXControlMode.PercentOutput, REJECT_OBJECT_SPEED_PCT))
            .withTimeout(REJECT_TIMEOUT_S)
            .andThen(() -> {
                turnOffCatchLed();
                stopMotor();
            });
    }

    private Command rejectCone() {
        return run(() -> openSolenoid())
            .withTimeout(REJECT_TIMEOUT_S)
            .andThen(() -> {
                turnOffCatchLed();
                stopMotor();
            });
    }

    private void turnOnCatchLed() {
        m_led.set(true);
    }

    private void turnOffCatchLed() {
        m_led.set(false);
    }

    private void stopMotor() { 
        m_motor.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
