package com.swervedrivespecialties.swervelib;

/**
 * An angle in the ]-inf, +inf[ range.
 */
public class ContinuousAngle {
    private final double m_angleDegrees;
    private final double m_angleRadians;

    private ContinuousAngle(double angleDegrees, double angleRadians) {
        m_angleDegrees = angleDegrees;
        m_angleRadians = angleRadians;
    }

    public static ContinuousAngle fromRadians(double angleRadians) {
        return new ContinuousAngle(Math.toDegrees(angleRadians), angleRadians);
    }

    public static ContinuousAngle fromDegrees(double angleDegrees) {
        return new ContinuousAngle(angleDegrees, Math.toRadians(angleDegrees));
    }
    
    public double radians() {
        return m_angleRadians;
    }

    public double degrees() {
        return m_angleDegrees;
    }

    public DiscreetAngle asDiscreet() {
        return DiscreetAngle.fromDegrees(m_angleDegrees);
    }

    public ContinuousAngle plus(ContinuousAngle other) {
        return new ContinuousAngle(m_angleDegrees + other.m_angleDegrees, m_angleRadians + other.m_angleRadians);
    }
}
