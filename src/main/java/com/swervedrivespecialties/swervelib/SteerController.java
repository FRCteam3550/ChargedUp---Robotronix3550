package com.swervedrivespecialties.swervelib;

public interface SteerController {
    ContinuousAngle getReferenceAngle();

    void setReferenceAngle(ContinuousAngle referenceAngle);

    /**
     * Range: ]-inf, +inf[
     */
    ContinuousAngle getStateAngle();
    DiscreetAngle getAbsoluteAngle();
}
