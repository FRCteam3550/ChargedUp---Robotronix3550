package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.swervedrivespecialties.swervelib.*;

public final class Falcon500SteerController implements SteerController {
    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

    private static final double TICKS_PER_ROTATION = 2048.0;
    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

    private final TalonFX motor;
    private final double motorEncoderPositionCoefficient;
    private final double motorEncoderVelocityCoefficient;
    private final TalonFXControlMode motorControlMode;
    private final AbsoluteEncoder absoluteEncoder;

    private ContinuousAngle referenceAngle = ContinuousAngle.fromDegrees(0);

    private double resetIteration = 0;

    public Falcon500SteerController(Falcon500SteerConfiguration steerConfiguration, GearRatio gearRatio) {
        motorEncoderPositionCoefficient = 2.0 * Math.PI / TICKS_PER_ROTATION * gearRatio.steerReduction;
        motorEncoderVelocityCoefficient = motorEncoderPositionCoefficient * 10.0;
        absoluteEncoder = steerConfiguration.absoluteEncoder;
        motorControlMode = steerConfiguration.hasMotionMagic() ? TalonFXControlMode.MotionMagic : TalonFXControlMode.Position;

        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        if (steerConfiguration.hasPidConstants()) {
            motorConfiguration.slot0.kP = steerConfiguration.proportionalConstant;
            motorConfiguration.slot0.kI = steerConfiguration.integralConstant;
            motorConfiguration.slot0.kD = steerConfiguration.derivativeConstant;
        }
        if (steerConfiguration.hasMotionMagic()) {
            if (steerConfiguration.hasVoltageCompensation()) {
                motorConfiguration.slot0.kF = (1023.0 * motorEncoderVelocityCoefficient / steerConfiguration.nominalVoltage) * steerConfiguration.velocityConstant;
            }
            // TODO: What should be done if no nominal voltage is configured? Use a default voltage?

            // TODO: Make motion magic max voltages configurable or dynamically determine optimal values
            motorConfiguration.motionCruiseVelocity = 2.0 / steerConfiguration.velocityConstant / motorEncoderVelocityCoefficient;
            motorConfiguration.motionAcceleration = (8.0 - 2.0) / steerConfiguration.accelerationConstant / motorEncoderVelocityCoefficient;
        }
        if (steerConfiguration.hasVoltageCompensation()) {
            motorConfiguration.voltageCompSaturation = steerConfiguration.nominalVoltage;
        }
        if (steerConfiguration.hasCurrentLimit()) {
            motorConfiguration.supplyCurrLimit.currentLimit = steerConfiguration.currentLimit;
            motorConfiguration.supplyCurrLimit.enable = true;
        }

        motor = new TalonFX(steerConfiguration.motorCANId);
        motor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS);

        if (steerConfiguration.hasVoltageCompensation()) {
            motor.enableVoltageCompensation(true);
        }
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS);
        motor.setSensorPhase(gearRatio.steerInverted);
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        motor.setNeutralMode(NeutralMode.Brake);

        motor.setSelectedSensorPosition(
                absoluteEncoder.getAbsoluteAngle().radians() / motorEncoderPositionCoefficient,
                0,
                CAN_TIMEOUT_MS);

        // Reduce CAN status frame rates
        motor.setStatusFramePeriod(
                StatusFrameEnhanced.Status_1_General,
                STATUS_FRAME_GENERAL_PERIOD_MS,
                CAN_TIMEOUT_MS
        );
    }

    @Override
    public ContinuousAngle getReferenceAngle() {
        return referenceAngle;
    }

    @Override
    public void setReferenceAngle(ContinuousAngle referenceAngle) {
        // Reset the NEO's absoluteEncoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute absoluteEncoder isn't fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter anymore.
        if (motor.getSelectedSensorVelocity() * motorEncoderVelocityCoefficient < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                resetIteration = 0;
                double absoluteAngle = absoluteEncoder.getAbsoluteAngle().radians();
                motor.setSelectedSensorPosition(absoluteAngle / motorEncoderPositionCoefficient);
            }
        } else {
            resetIteration = 0;
        }
        motor.set(motorControlMode, referenceAngle.radians() / motorEncoderPositionCoefficient);


        this.referenceAngle = referenceAngle;
    }

    @Override
    public ContinuousAngle getStateAngle() {
        return ContinuousAngle.fromRadians(motor.getSelectedSensorPosition() * motorEncoderPositionCoefficient);
    }

    @Override
    public DiscreetAngle getAbsoluteAngle() {
        return absoluteEncoder.getAbsoluteAngle();
    }

}
