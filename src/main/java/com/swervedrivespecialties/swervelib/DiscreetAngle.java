package com.swervedrivespecialties.swervelib;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * An angle in the [0, 360[ range.
 */
public class DiscreetAngle {
    private static final double TWO_PI = 2 * Math.PI;
    private final double m_angleDegrees;
    private final double m_angleRadians;

    private DiscreetAngle(double angleDegrees, double angleRadians) {
        m_angleDegrees = angleDegrees;
        m_angleRadians = angleRadians;
    }

    public static DiscreetAngle fromRadians(double angleRadians) {
        var discreetRadians = angleRadians % TWO_PI;
        discreetRadians = discreetRadians < 0 ? discreetRadians + TWO_PI : discreetRadians;
        return new DiscreetAngle(Math.toDegrees(discreetRadians), discreetRadians);
    }

    public static DiscreetAngle fromDegrees(double angleDegrees) {
        var discreetDegrees = angleDegrees % 360;
        discreetDegrees = discreetDegrees < 0 ? discreetDegrees + 360 : discreetDegrees;
        return new DiscreetAngle(discreetDegrees, Math.toRadians(discreetDegrees));
    }
    
    public static DiscreetAngle fromRotation(Rotation2d rotation) {
        return fromDegrees(rotation.getDegrees());
    }

    public double radians() {
        return m_angleRadians;
    }

    public double degrees() {
        return m_angleDegrees;
    }
}
