package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PilotShuffleboardLayout;
import frc.robot.utils.AtReference;

public class Elevator extends SubsystemBase {
    enum InitPhase {
        FIND_ZERO,
        FINISHED
    }

    private class goToHeightCommand extends CommandBase {
        private boolean areWeGoingUp;
        private DigitalInput limitSwitch;
        private double height;

        public goToHeightCommand(double height) {
            this.height = height;

            addRequirements(Elevator.this);
        }

        @Override
        public void initialize() {
            areWeGoingUp = (height - m_encoderLeft.getPosition()) > 0;
            limitSwitch = areWeGoingUp ? m_topLimitSwitch : m_bottomLimitSwitch;
        }

        @Override
        public void execute() {
            m_pidController.setReference(height, CANSparkMax.ControlType.kPosition);
        }

        @Override
        public void end(boolean interrupted) {
            stopMotor();
        }

        @Override
        public boolean isFinished() {
            return m_referenceUtil.atSetpoint(height) || isActivated(limitSwitch);
        }
    }

    private InitPhase m_phase = InitPhase.FIND_ZERO;

    private static final double INIT_DOWN_SPEED = -0.04; 


    //Mesure sur le cote gauche (le bout de l'extenseur et la petite plaque) et avec l'encodeur gauche
    private static final double BOTTOM_METERS = 0.39;
    private static final double BOTTOM_TICKS = .386;
    
    private static final double TOP_METERS = 0.973;
    private static final double TOP_TICKS = 108.9352;

    private static final double ENCODER_ZERO_TO_GROUND_METERS = 0.39;

    private static final double TICKS_TO_METERS_COEFFICIENT = (TOP_METERS - BOTTOM_METERS) / (TOP_TICKS - BOTTOM_TICKS);

    private static final double K_P = 40; // 75 
    private static final double K_I = 0; 
    private static final double K_D = 0; // 10000
    
    private static final double OUTPUT_RANGE_MIN = -1; 
    private static final double OUTPUT_RANGE_MAX = 1; 

    private static final double BOTTOM_GRID_ROW = ENCODER_ZERO_TO_GROUND_METERS;
    private static final double MEDIUM_GRID_ROW_CONE = 0.805192; 
    private static final double MEDIUM_GRID_ROW_CUBE = ENCODER_ZERO_TO_GROUND_METERS;
    private static final double TOP_GRID_ROW_CONE = 1.047162;
    private static final double TOP_GRID_ROW_CUBE = 0.827083;
    private static final double LOADING_ZONE_TABLET = 1.028;
    private static final double FALLEN_CONE = 0.694;
    private static final double TEST_LENGHT = .75;
    private static final double TEST_LENGTH_FOURTY = ENCODER_ZERO_TO_GROUND_METERS + 0.40;
    private static final double TEST_LENGTH_TWENTY = ENCODER_ZERO_TO_GROUND_METERS + 0.20;

    private static final double ERROR_TOLERENCE = 0.05;// 0.015 

    private final CANSparkMax m_motorRight = new CANSparkMax(Constants.Elevator.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax m_motorLeft = new CANSparkMax(Constants.Elevator.LEFT_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder m_encoderRight = m_motorRight.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
            Constants.NeoMotor.TICKS_PER_REV);
    private final RelativeEncoder m_encoderLeft = m_motorLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
            Constants.NeoMotor.TICKS_PER_REV);

    private final DigitalInput m_bottomLimitSwitch = new DigitalInput(Constants.Elevator.BOTTOM_LIMITSWITCH_ID);
    private final DigitalInput m_topLimitSwitch = new DigitalInput(Constants.Elevator.TOP_LIMITSWITCH_ID);

    private final SparkMaxPIDController m_pidController = m_motorLeft.getPIDController();
    private AtReference m_referenceUtil = new AtReference(ERROR_TOLERENCE, m_encoderLeft);

    // Ratio de transmition: 28

    public Elevator() {
        m_motorLeft.setInverted(true);
        m_motorRight.follow(m_motorLeft, true);

        m_encoderLeft.setPositionConversionFactor(TICKS_TO_METERS_COEFFICIENT);
        m_encoderRight.setPositionConversionFactor(TICKS_TO_METERS_COEFFICIENT);

        m_encoderLeft.setPosition(ENCODER_ZERO_TO_GROUND_METERS);
        m_encoderRight.setPosition(ENCODER_ZERO_TO_GROUND_METERS);

        m_pidController.setP(K_P);
        m_pidController.setI(K_I);
        m_pidController.setD(K_D);

        m_pidController.setOutputRange(OUTPUT_RANGE_MIN, OUTPUT_RANGE_MAX);

        PilotShuffleboardLayout.MECHANISMS_LAYOUT.addDouble("Hauteur m", m_encoderLeft::getPosition);
        PilotShuffleboardLayout.LS_LAYOUT.addBoolean("Elevateur Init", () -> isActivated(m_bottomLimitSwitch));

        m_motorLeft.setIdleMode(IdleMode.kBrake);
        m_motorRight.setIdleMode(IdleMode.kBrake);

        stopMotor();
    }

    private boolean isActivated(DigitalInput limitSwitch) {
        return !limitSwitch.get();
    }

    public Command init() {
        return run(() -> {
            if (m_phase != InitPhase.FINISHED) {
                if (isActivated(m_bottomLimitSwitch)) {
                    stopMotor();
                    m_encoderLeft.setPosition(ENCODER_ZERO_TO_GROUND_METERS);
                    m_encoderRight.setPosition(ENCODER_ZERO_TO_GROUND_METERS);
                    m_phase = InitPhase.FINISHED;
                }
                else {
                    m_motorLeft.set(INIT_DOWN_SPEED);
                }
            }
        })
        .until(() -> m_phase == InitPhase.FINISHED);
    }

    private void stopMotor() {
        m_motorLeft.set(0);
    }

    public Command goToHeight(double height) {
        return new goToHeightCommand(height);
    }

    public Command switchBrake() {
        return Commands.runOnce(() -> {
            m_motorLeft.setIdleMode(IdleMode.kBrake);
            m_motorRight.setIdleMode(IdleMode.kBrake);
        }).withName("switchBrake");
    }

    public Command switchCoast() {
        return Commands.runOnce(() -> {
            m_motorLeft.setIdleMode(IdleMode.kCoast);
            m_motorRight.setIdleMode(IdleMode.kCoast);
        }).withName("switchCoast");
    }

    public Command goToTopGridRowCone() {
        return goToHeight(TOP_GRID_ROW_CONE).withName("goToTopGridRowCone");
    }

    public Command goToTopGridRowCube() {
        return goToHeight(TOP_GRID_ROW_CUBE).withName("goToTopGridRowCube");
    }

    public Command goToMediumGridRowCone() {
        return goToHeight(MEDIUM_GRID_ROW_CONE).withName("goToMediumGridRowCone");
    }

    public Command goToMediumGridRowCube() {
        return goToHeight(MEDIUM_GRID_ROW_CUBE).withName("goToMediumGridRowCube");
    }

    public Command goToBottom() {
        return goToHeight(BOTTOM_GRID_ROW).withName("goToBottom");
    }

    public Command goToLoadingZoneTablet() {
        return goToHeight(LOADING_ZONE_TABLET).withName("goToLoadingZoneTablet");
    }

    public Command goToFallenCone() {
        return goToHeight(FALLEN_CONE).withName("goToFallenCone");
    }

    public Command goToFourthCM() {
        return goToHeight(TEST_LENGTH_FOURTY).withName("goToFourtyCM");
    }

    public Command goToTwentyCM() {
        return goToHeight(TEST_LENGTH_TWENTY).withName("goToTwentyCM");
    }

    public Command test() {
        return goToHeight(TEST_LENGHT);
    }
}
