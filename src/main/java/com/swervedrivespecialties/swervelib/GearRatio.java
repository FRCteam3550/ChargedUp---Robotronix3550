package com.swervedrivespecialties.swervelib;

/**
 * A swerve module configuration.
 * <p>
 * A configuration represents a unique mechanical configuration of a module. For example, the Swerve Drive Specialties
 * Mk3 swerve module has two configurations, standard and fast, and therefore should have two configurations
 * ({@link SdsGearRatios#MK3_STANDARD} and {@link SdsGearRatios#MK3_FAST} respectively).

 * @param wheelCircumferenceM  The distance the module's wheel travel per turn in meters.
 * @param driveReduction The overall drive reduction of the module. Multiplying motor rotations by this value
 *                       should result in wheel rotations.
 * @param driveInverted  Whether the drive motor should be inverted. If there is an odd number of gea reductions
 *                       this is typically true.
 * @param steerReduction The overall steer reduction of the module. Multiplying motor rotations by this value
 *                       should result in rotations of the steering pulley.
 * @param steerInverted  Whether the steer motor should be inverted. If there is an odd number of gear reductions
 *                       this is typically true.
 */
public class GearRatio {
    public final double wheelCircumferenceM;
    public final double driveReduction;
    public final boolean driveInverted;
    public final double steerReduction;
    public final boolean steerInverted;

    public GearRatio(double wheelCircumferenceM, double driveReduction, boolean driveInverted, double steerReduction, boolean steerInverted) {
        this.wheelCircumferenceM = wheelCircumferenceM;
        this.driveReduction = driveReduction;
        this.driveInverted = driveInverted;
        this.steerReduction = steerReduction;
        this.steerInverted = steerInverted;
    }

    public GearRatio withWheelCircumference(double wheelCircumferenceM) {
        return new GearRatio(wheelCircumferenceM, driveReduction, driveInverted, steerReduction, steerInverted);
    }
}
