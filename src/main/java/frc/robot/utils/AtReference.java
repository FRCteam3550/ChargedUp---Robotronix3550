package frc.robot.utils;

import com.revrobotics.RelativeEncoder;

public class AtReference {
    private final double m_errorTolerence;
    private final RelativeEncoder m_encoder;

    public AtReference(double errorTolerence, RelativeEncoder encoder) {
        this.m_errorTolerence = errorTolerence;
        this.m_encoder = encoder;
    }

    public boolean atSetpoint(double reference) {
        var error = reference - m_encoder.getPosition();
        
        return Math.abs(error) < m_errorTolerence;
    }
}
