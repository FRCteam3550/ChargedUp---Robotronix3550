package com.swervedrivespecialties.swervelib;

public interface DriveController {
    /**
     * Sets the motor's output.
     * @param pct a number between -1 and 1.
     */
    void setReferencePct(double pct);

    void setReferenceSpeedMS(double speedMS);

    double getReferenceSpeedMS();

    double getStateVelocityMS();

    double getStatePositionM();
}
