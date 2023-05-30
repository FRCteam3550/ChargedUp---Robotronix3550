package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.DiscreetAngle;

public class CANCoderAbsoluteEncoder implements AbsoluteEncoder {
    private static final int TIMEOUT_MS = 250;
    private static final boolean COUNTER_CLOCKWISE = false;
    private final CANCoder m_encoder;

    public CANCoderAbsoluteEncoder(int CANId, DiscreetAngle alignAngle, int readingUpdatePeriodMs) {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = -alignAngle.degrees();
        config.sensorDirection = COUNTER_CLOCKWISE;

        m_encoder = new CANCoder(CANId);
        m_encoder.configAllSettings(config, TIMEOUT_MS);
        m_encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, readingUpdatePeriodMs, TIMEOUT_MS);
    }
    @Override
    public DiscreetAngle getAbsoluteAngle() {
        return DiscreetAngle.fromDegrees(m_encoder.getAbsolutePosition());
    }

}
