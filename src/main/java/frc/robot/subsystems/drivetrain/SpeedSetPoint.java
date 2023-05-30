package frc.robot.subsystems.drivetrain;

public class SpeedSetPoint {
    public final double xSpeed;
    public final double ySpeed;
    public final double rotationSpeed;

    /**
     * A drivetrain speed set point.
     * 
     * Units can vary depending on the producer or consumer of this class. It could be in percentage or real speed units.
     * Reference coordinates vary as well. Could be field reference or robot reference.
     * 
     * @param xSpeed the desired translation speed in the X axis.
     * @param ySpeed the desired translation speed in the Y axis.
     * @param rotationSpeed the desired rotation speed.
     */
    public SpeedSetPoint(double xSpeed, double ySpeed, double rotationSpeed) {
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotationSpeed = rotationSpeed;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        long temp;
        temp = Double.doubleToLongBits(xSpeed);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(ySpeed);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(rotationSpeed);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null || getClass() != obj.getClass())
            return false;

        SpeedSetPoint other = (SpeedSetPoint) obj;
        return xSpeed == other.xSpeed && ySpeed == other.ySpeed && rotationSpeed == other.rotationSpeed;
    }

    @Override
    public String toString() {
        return String.format("x: %.4f y:%.4f rot:%.4f", xSpeed, ySpeed, rotationSpeed);
    }
}
