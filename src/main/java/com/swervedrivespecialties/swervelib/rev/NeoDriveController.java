package com.swervedrivespecialties.swervelib.rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.GearRatio;

public final class NeoDriveController implements DriveController {
    private final CANSparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private double m_referenceSpeedMS = 0;

    public NeoDriveController(NeoDriveConfiguration configuration, GearRatio gearRatio) {
        m_motor = new CANSparkMax(configuration.motorCANId, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_motor.setInverted(gearRatio.driveInverted);

        // Setup voltage compensation
        if (configuration.hasVoltageCompensation()) {
            m_motor.enableVoltageCompensation(configuration.nominalVoltage);
        }

        if (configuration.hasCurrentLimit()) {
            m_motor.setSmartCurrentLimit((int)configuration.currentLimit);
        }

        m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        m_motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
        // Set neutral mode to brake
        m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Setup absoluteEncoder
        m_encoder = m_motor.getEncoder();
        double positionConversionFactor = gearRatio.wheelCircumferenceM * gearRatio.driveReduction;
        m_encoder.setPositionConversionFactor(positionConversionFactor);
        m_encoder.setVelocityConversionFactor(positionConversionFactor / 60.0);
    }

    @Override
    public void setReferencePct(double pct) {
        m_motor.set(pct);
    }

    @Override
    public void setReferenceSpeedMS(double speedMS) {
        m_referenceSpeedMS = speedMS;
        throw new UnsupportedOperationException();
    }

    @Override
    public double getStateVelocityMS() {
        return m_encoder.getVelocity();
    }

    @Override
    public double getStatePositionM() {
        return m_encoder.getPosition();
    }

    @Override
    public double getReferenceSpeedMS() {
        return m_referenceSpeedMS;
    }

}
