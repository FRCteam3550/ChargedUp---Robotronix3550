package com.swervedrivespecialties.swervelib;

public interface SwerveModule {
    static final double TWO_PI = 2 * Math.PI;
    static final double HALF_PI = Math.PI / 2.0;

    public class SteerSetPoint {
        public final double driveSign;
        public final ContinuousAngle targetAngle;
        public SteerSetPoint(double driveSign, ContinuousAngle targetAngle) {
            this.driveSign = driveSign;
            this.targetAngle = targetAngle;
        }
    }

    double getDriveVelocity();

    double getDrivePosition();

    ContinuousAngle getSteerAngle();

    /**
     * Sets the motors instructions.
     * @param drivePct a number between -1 and 1.
     * @param steerAngle the angle for the steer module to go to.
     */
    void set(double drivePct, DiscreetAngle steerAngle);
    void setMS(double driveMS, DiscreetAngle steerAngle);

    static SteerSetPoint getSteerAngleAndDriveSign(DiscreetAngle targetAngle, ContinuousAngle currentAngle) {
        var driveSign = 1.0;

        double difference = targetAngle.radians() - currentAngle.asDiscreet().radians();
        // Change the target angle so the difference is in the range [-pi, pi) instead of [-2pi, 2pi)
        if (difference >= Math.PI) {
            difference -= TWO_PI;
        } else if (difference < -Math.PI) {
            difference += TWO_PI;
        }

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (difference > HALF_PI) {
            difference -= Math.PI;
            driveSign = -1.0;
        } else if (difference < -HALF_PI) {
            difference += Math.PI;
            driveSign = -1.0;
        }

        return new SteerSetPoint(
            driveSign, 
            currentAngle.plus(ContinuousAngle.fromRadians(difference))
        );
    }
}
