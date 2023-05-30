package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.GearRatio;

import frc.robot.Constants;

public final class Falcon500DriveController implements DriveController {
    private static final double TICKS_PER_ROTATION = 2048.0;

    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
    private final TalonFX m_motor;
    private final double m_sensorVelocityCoefficient;
    private final double m_metersPerTicks;
    private double m_referenceSpeedMS = 0;

    public Falcon500DriveController(Falcon500DriveConfiguration configuration, GearRatio gearRatio) {
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        m_metersPerTicks = gearRatio.wheelCircumferenceM * gearRatio.driveReduction / TICKS_PER_ROTATION;
        m_sensorVelocityCoefficient = m_metersPerTicks * 10.0;

        if (configuration.hasPidConstants()) {
            if (Double.isNaN(configuration.feedForwardConstant)) {
                var maxTicksPerSeconds = Constants.TEMP_MAX_SPEED_MS / m_metersPerTicks;
                var maxTicksPer100ms = maxTicksPerSeconds / 10;
                motorConfiguration.slot0.kF = 1023 / maxTicksPer100ms;
            } else {
                motorConfiguration.slot0.kF = configuration.feedForwardConstant;
            }
            motorConfiguration.slot0.kP = configuration.proportionalConstant;
            motorConfiguration.slot0.kI = configuration.integralConstant;
            motorConfiguration.slot0.kD = configuration.derivativeConstant;
        }

        if (configuration.hasVoltageCompensation()) {
            motorConfiguration.voltageCompSaturation = configuration.nominalVoltage;
        }

        if (configuration.hasCurrentLimit()) {
            motorConfiguration.supplyCurrLimit.currentLimit = configuration.currentLimit;
            motorConfiguration.supplyCurrLimit.enable = true;
        }

        m_motor = new TalonFX(configuration.motorCANId);
        m_motor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS);

        if (configuration.hasVoltageCompensation()) {
            // Enable voltage compensation
            m_motor.enableVoltageCompensation(true);
        }

        m_motor.setNeutralMode(NeutralMode.Brake);

        m_motor.setInverted(gearRatio.driveInverted ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
        m_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS);
        m_motor.setSensorPhase(true);

        // Reduce CAN status frame rates
        m_motor.setStatusFramePeriod(
                StatusFrameEnhanced.Status_1_General,
                STATUS_FRAME_GENERAL_PERIOD_MS,
                CAN_TIMEOUT_MS
        );
    }

    @Override
    public void setReferencePct(double pct) {
        m_motor.set(TalonFXControlMode.PercentOutput, pct);
    }

    @Override
    public void setReferenceSpeedMS(double speedMS) {
        m_motor.set(TalonFXControlMode.Velocity, speedMS / m_sensorVelocityCoefficient);
        m_referenceSpeedMS = speedMS;
    }

    @Override
    public double getStateVelocityMS() {
        return m_motor.getSelectedSensorVelocity() * m_sensorVelocityCoefficient;
    }

    @Override
    public double getStatePositionM() {
        return m_motor.getSelectedSensorPosition() * m_metersPerTicks;
    }

    @Override
    public double getReferenceSpeedMS() {
        return m_referenceSpeedMS;
    }
}
