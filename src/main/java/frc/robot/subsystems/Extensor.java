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

public class Extensor extends SubsystemBase {
    enum InitPhase {
        FIND_ZERO,
        FINISHED
    }

    private class goToLengthCommand extends CommandBase {
        private boolean areWeExtending;
        private DigitalInput limitSwitch;
        private double length;

        public goToLengthCommand(double length) {
            this.length = length;
            addRequirements(Extensor.this);
        }

        @Override
        public void initialize() {
            areWeExtending = (length - m_encoder.getPosition()) > 0;
            limitSwitch = areWeExtending ? m_extendedLimitSwitch : m_retractedLimitSwitch; 
        }

        @Override
        public void execute() {
            m_pidController.setReference(length, CANSparkMax.ControlType.kPosition);
        }

        @Override
        public void end(boolean interrupted) {
            stopMotor();
        }

        @Override
        public boolean isFinished() {
            return m_referenceUtil.atSetpoint(length) || isActivated(limitSwitch);
        }
    }

    private InitPhase m_phase = InitPhase.FIND_ZERO;

    private static final double EXTENDED_METERS = 0.623; 
    private static final double EXTENDED_TICKS = 155.8914; 

    private static final double RETRACTED_METERS = 0.127; 
    private static final double RETRACTED_TICKS = 0.134; 

    private static final double ENCODER_RETRACTED_METERS = .127; 

    private static final double TICKS_TO_METERS_COEFFICIENT = (EXTENDED_METERS - RETRACTED_METERS)
            / (EXTENDED_TICKS - RETRACTED_TICKS);

    private static final double K_P = 40; 
    private static final double K_I = 0;  
    private static final double K_D = 0; 

    private static final double OUTPUT_RANGE_MIN = -1;
    private static final double OUTPUT_RANGE_MAX = 1;

    private static final double INIT_SPEED = 0.4;

    private static final double TARGET_HIGH_POSITION_CONE = 0.552043;
    private static final double TARGET_HIGH_POSITION_CUBE = 0.382591;
    private static final double TARGET_RETRACTED_POSITION_M = ENCODER_RETRACTED_METERS;
    private static final double TARGET_MIDDLE_POSITION_CONE_M = ENCODER_RETRACTED_METERS + 0.04;
    private static final double TARGET_MIDDLE_POSITION_CUBE_M = ENCODER_RETRACTED_METERS;
    private static final double TARGET_LOADING_ZONE_TABLET_POSITION_M = ENCODER_RETRACTED_METERS;
    private static final double FALLEN_CONE = 0.264;
    private static final double TEST_LENGHT = .43;

    private static final double ERROR_TOLERENCE = 0.015;

    private final CANSparkMax m_motor = new CANSparkMax(Constants.Extensor.MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
            Constants.NeoMotor.TICKS_PER_REV);
    private final DigitalInput m_retractedLimitSwitch = new DigitalInput(Constants.Extensor.RETRACTED_LIMITSWITCH_ID);
    private final DigitalInput m_extendedLimitSwitch = new DigitalInput(Constants.Extensor.EXTENDED_LIMITSWITCH_ID);
    private final SparkMaxPIDController m_pidController = m_motor.getPIDController();
    private AtReference m_referenceUtil = new AtReference(ERROR_TOLERENCE, m_encoder);

    // Ratio de transmition: 45

    public Extensor() {
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(true);

        m_encoder.setPositionConversionFactor(TICKS_TO_METERS_COEFFICIENT);
        m_encoder.setPosition(ENCODER_RETRACTED_METERS);

        m_pidController.setP(K_P);
        m_pidController.setI(K_I);
        m_pidController.setD(K_D);
        m_pidController.setOutputRange(OUTPUT_RANGE_MIN, OUTPUT_RANGE_MAX);

        PilotShuffleboardLayout.MECHANISMS_LAYOUT.addDouble("Extension m", m_encoder::getPosition);
        PilotShuffleboardLayout.LS_LAYOUT.addBoolean("Extenseur Init", () -> isActivated(m_retractedLimitSwitch));

        stopMotor();
    }

    public Command init() {
        return run(() -> {
            if (m_phase != InitPhase.FINISHED) {
                if (isActivated(m_retractedLimitSwitch)) {
                    stopMotor();
                    m_encoder.setPosition(ENCODER_RETRACTED_METERS);
                    m_phase = InitPhase.FINISHED;
                } else {
                    m_motor.set(INIT_SPEED);
                }
            }
        })
        .until(() -> m_phase == InitPhase.FINISHED);
    }

    private boolean isActivated(DigitalInput limitSwitch) {
        return !limitSwitch.get();
    }

    public Command goToLength(double length) {
        return new goToLengthCommand(length);
    } 

    private void stopMotor() {
        m_motor.set(0);
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

    public Command goToTopGridRowCone() {
        return goToLength(TARGET_HIGH_POSITION_CONE).withName("goToTopGridRowCone");
    }

    public Command goToTopGridRowCube() {
        return goToLength(TARGET_HIGH_POSITION_CUBE).withName("goToTopGridRowCube");
    }

    public Command goToMediumGridRowCone() {
        return goToLength(TARGET_MIDDLE_POSITION_CONE_M).withName("goToMediumGridRowCone");
    }

    public Command goToMediumGridRowCube() {
        return goToLength(TARGET_MIDDLE_POSITION_CUBE_M).withName("goToMediumGridRowCube");
    }

    public Command goInside() {
        return goToLength(TARGET_RETRACTED_POSITION_M).withName("goInside");
    }

    public Command goToLoadingZoneTablet() {
        return goToLength(TARGET_LOADING_ZONE_TABLET_POSITION_M).withName("goToLoadingZoneTablet");
    }

    public Command goToFallenCone() {
        return goToLength(FALLEN_CONE).withName("goToFallenCone");
    }

    public Command test() {
        return goToLength(TEST_LENGHT);
    }
}
