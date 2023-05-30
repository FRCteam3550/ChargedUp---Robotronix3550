package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PilotShuffleboardLayout;
import frc.robot.utils.AtReference;

public class Pivot extends SubsystemBase {
    enum InitPhase {
        FIND_ZERO,
        FINISHED
    }

    private class goToAngleCommand extends CommandBase {
        // private boolean areWeGoingDeg;
        // private DigitalInput limitSwitch;
        private double rotAngleDeg;
        private boolean stopMotorWhenFinished;

        public goToAngleCommand(double rotAngleDeg, boolean stopMotorWhenFinished) {
            this.rotAngleDeg = rotAngleDeg;
            this.stopMotorWhenFinished = stopMotorWhenFinished;
            addRequirements(Pivot.this);
        }

        @Override
        public void initialize() {
            // areWeGoingDeg = (rotAngleDeg - m_encoder.getPosition()) > 0;
            // limitSwitch = areWeGoingDeg ? m_extendedSwitch : m_retractedSwitch;
        }

        @Override
        public void execute() {
            m_pidController.setReference(rotAngleDeg, CANSparkMax.ControlType.kPosition);
        }

        @Override
        public void end(boolean interrupted) {
            if (stopMotorWhenFinished) {
                stopMotor();
            }
        }

        @Override
        public boolean isFinished() {
            return m_referenceUtil.atSetpoint(rotAngleDeg) /*|| isActivated(limitSwitch)*/;
        }
    }
    
    private InitPhase m_phase = InitPhase.FIND_ZERO;

    private static final double INIT_RETRACTED_SPEED = -0.04; 

    private static final double UP_TICKS = 1.642858;
    private static final double HORIZONTAL_TICKS = 15;
    private static final double ENCODER_RETRACTED_ANGLE_DEGREES = -99.625717;
    private static final double ANGLE_TO_TICKS_COEFFICIENT = 90 / (HORIZONTAL_TICKS - UP_TICKS);
    
    private static final double K_P = 0.1;
    private static final double K_I = 0;
    private static final double K_D = 0;
    
    private static final double OUTPUT_RANGE_MIN = -.2;
    private static final double OUTPUT_RANGE_MAX = .2;

    private static final double RETRACTED_ANGLE = ENCODER_RETRACTED_ANGLE_DEGREES;
    private static final double HORIZONTAL_ANGLE = 0;
    private static final double DOWN_ANGLE = 90;
    private static final double MIDDLE_ANGLE_CONE = -35.133648;
    private static final double MIDDLE_ANGLE_CUBE = -47.647125;
    private static final double HIGH_ANGLE_CONE = -50;
    private static final double HIGH_ANGLE_CUBE = -36.577492;
    private static final double UP_ANGLE = -90;
    private static final double SUBSTATION_CONE_ANGLE_DEG = -15.401065;
    private static final double SUBSTATION_CUBE_ANGLE_DEG = -16.844913;
    private static final double FLOOR_ANGLE_CUBE = 10.5; // etais a 9.465248
    private static final double FLOOR_ANGLE_CONE = 13.5; // etais a 9.465248

    private static final double RETRACTED_ANGLE_SUBSTATION_CONE = -93.369133;
    
    private final CANSparkMax m_motor = new CANSparkMax(Constants.Pivot.MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.NeoMotor.TICKS_PER_REV);
     // Ratio de transmition: (9/1) * (5/1) * (30/12)

    private final DigitalInput m_retractedSwitch = new DigitalInput(Constants.Pivot.RETRACTED_LIMITSWITCH_ID);

    private final SparkMaxPIDController m_pidController = m_motor.getPIDController();
    private static final double ERROR_TOLERANCE = 2;

    private AtReference m_referenceUtil = new AtReference(ERROR_TOLERANCE, m_encoder);

    private boolean m_cancelPid = false;

    public Pivot() {
        m_motor.setInverted(true);
        m_motor.setIdleMode(IdleMode.kBrake);
        
        m_encoder.setPositionConversionFactor(ANGLE_TO_TICKS_COEFFICIENT);
        m_encoder.setPosition(ENCODER_RETRACTED_ANGLE_DEGREES);

        m_pidController.setP(K_P);
        m_pidController.setI(K_I);
        m_pidController.setD(K_D);
        m_pidController.setOutputRange(OUTPUT_RANGE_MIN, OUTPUT_RANGE_MAX);

        PilotShuffleboardLayout.MECHANISMS_LAYOUT.addDouble("Angle d", m_encoder::getPosition);
        PilotShuffleboardLayout.LS_LAYOUT.addBoolean("Pivot Init", () -> isActivated(m_retractedSwitch));

        stopMotor();
    }

    private boolean isActivated(DigitalInput limitSwitch) {
        return !limitSwitch.get();
    }

    public Command init() {
        return run(() -> {
            if (m_phase != InitPhase.FINISHED) {
                if (isActivated(m_retractedSwitch)) {
                    stopMotor();
                    m_encoder.setPosition(ENCODER_RETRACTED_ANGLE_DEGREES);
                    m_phase = InitPhase.FINISHED;
                } else {
                    m_motor.set(INIT_RETRACTED_SPEED);
                    m_phase = InitPhase.FIND_ZERO;
                }
            }
        })
        .until(() -> m_phase == InitPhase.FINISHED);
    }

    private void stopMotor() {
        m_motor.set(0);
    }

    public Command cancelPid() {
        return Commands.runOnce(() -> {
            m_cancelPid = true;
        });
    }

    public Command goToRotAngle(double rotAngle) {
        return new goToAngleCommand(rotAngle, true);
    } 

    public Command goToRotAngleAndStayActive(double rotAngle) {
        return new goToAngleCommand(rotAngle, false);
    } 

    public Command switchBrake() {
        return Commands.runOnce(() -> {
            m_motor.setIdleMode(IdleMode.kBrake);
        }).withName("switchBrake");
    }

    public Command switchCoast() {
        return Commands.runOnce(() -> {
            m_motor.setIdleMode(IdleMode.kCoast);
        }).withName("switchCoast");
    }

    public Command goToUp() {
        return goToRotAngle(UP_ANGLE).withName("goToUp");
    }

    public Command goToHorizontal() {
        return goToRotAngleAndStayActive(HORIZONTAL_ANGLE).withName("goToHorizontal");
    }

    public Command goToFloorCube() {
        return goToRotAngleAndStayActive(FLOOR_ANGLE_CUBE).withName("goToFloorCube");
    }

    public Command goToFloorCone() {
        return goToRotAngleAndStayActive(FLOOR_ANGLE_CONE).withName("goToFloorCone");
    }

    public Command goToRetractedAngle() {
        return goToRotAngle(RETRACTED_ANGLE).withName("goToRetractedAngle");
    }

    public Command goToDownAngle() { 
        return goToRotAngle(DOWN_ANGLE).withName("goToDownAngle");
    }

    public Command goToHighRowCone() { 
        return goToRotAngleAndStayActive(HIGH_ANGLE_CONE).withName("goToHighRowCone");
    }

    public Command goToHighRowCube() { 
        return goToRotAngleAndStayActive(HIGH_ANGLE_CUBE).withName("goToHighRowCone");
    }

    public Command goToMiddleRowCone() { 
        return goToRotAngleAndStayActive(MIDDLE_ANGLE_CONE).withName("goToMidlleRowCone");
    }

    public Command goToMiddleRowCube() { 
        return goToRotAngleAndStayActive(MIDDLE_ANGLE_CUBE).withName("goToMiddleRowCube");
    }

    public Command goToSubstationCone() { 
        return goToRotAngleAndStayActive(SUBSTATION_CONE_ANGLE_DEG).withName("goToSubstationCone");
    }

    public Command goToSubstationCube() { 
        return goToRotAngleAndStayActive(SUBSTATION_CUBE_ANGLE_DEG).withName("goToSubstationCube");
    }

    public Command goToRetractedAngleSubstationCone() { 
        return goToRotAngle(RETRACTED_ANGLE_SUBSTATION_CONE).withName("goToRetractedAngleSubstationCone");
    }
}

