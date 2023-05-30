package com.swervedrivespecialties.swervelib.rev;

public class NeoDriveConfiguration {
    private static final double DEFAULT_NOMINAL_VOLTAGE = 12;
    private static final double DEFAULT_DRIVE_CURRENT_LIMIT = 80;

    public final int motorCANId;
    public final double nominalVoltage;
    public final double currentLimit;

    public NeoDriveConfiguration(int motorCANId, double nominalVoltage, double currentLimit) {
        this.motorCANId = motorCANId;
        this.nominalVoltage = nominalVoltage;
        this.currentLimit = currentLimit;
    }

    public NeoDriveConfiguration(int motorCANId) {
        this(motorCANId, DEFAULT_NOMINAL_VOLTAGE, DEFAULT_DRIVE_CURRENT_LIMIT);
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public NeoDriveConfiguration withVoltageCompensation(double nominalVoltage) {
        return new NeoDriveConfiguration(this.motorCANId, nominalVoltage, this.currentLimit);
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public NeoDriveConfiguration withCurrentLimit(double currentLimit) {
        return new NeoDriveConfiguration(this.motorCANId, this.nominalVoltage, currentLimit);
    }
}
