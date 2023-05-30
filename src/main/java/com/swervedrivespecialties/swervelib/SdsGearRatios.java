package com.swervedrivespecialties.swervelib;

public final class SdsGearRatios {
    public static final GearRatio MK4_L1 = new GearRatio(
            0.10033 * Math.PI, // 0.3152m
            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
            false,
            (15.0 / 32.0) * (10.0 / 60.0),
            true
    );
    public static final GearRatio MK4_L2 = new GearRatio(
            0.10033 * Math.PI,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true
    );
    public static final GearRatio MK4_L3 = new GearRatio(
            0.10033 * Math.PI,
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true
    );
    public static final GearRatio MK4_L4 = new GearRatio(
            0.10033 * Math.PI,
            (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true
    );

    private SdsGearRatios() {
    }
}
