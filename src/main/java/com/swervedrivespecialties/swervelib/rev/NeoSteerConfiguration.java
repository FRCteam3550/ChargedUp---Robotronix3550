package com.swervedrivespecialties.swervelib.rev;

import com.swervedrivespecialties.swervelib.AbsoluteEncoder;

public class NeoSteerConfiguration {
    private static final double DEFAULT_NOMINAL_VOLTAGE = 12;
    private static final double DEFAULT_CURRENT_LIMIT = 20;

    public final int motorCANId;
    public final AbsoluteEncoder absoluteEncoder;
    public final double nominalVoltage;
    public final double currentLimit;
    public final double proportionalConstant;
    public final double integralConstant;
    public final double derivativeConstant;

    public NeoSteerConfiguration(
        int motorCANId,
        AbsoluteEncoder absoluteEncoder,
        double nominalVoltage,
        double currentLimit,
        double proportionalConstant,
        double integralConstant,
        double derivativeConstant) {
        this.motorCANId = motorCANId;
        this.absoluteEncoder = absoluteEncoder;
        this.nominalVoltage = nominalVoltage;
        this.currentLimit = currentLimit;
        this.proportionalConstant = proportionalConstant;
        this.integralConstant = integralConstant;
        this.derivativeConstant = derivativeConstant;
    }

    public NeoSteerConfiguration(int motorCANId, AbsoluteEncoder absoluteEncoder) {
        this(
                motorCANId,
                absoluteEncoder,
                DEFAULT_NOMINAL_VOLTAGE,
                DEFAULT_CURRENT_LIMIT,
                Double.NaN,
                Double.NaN,
                Double.NaN
        );
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public NeoSteerConfiguration withVoltageCompensation(double nominalVoltage) {
        return new NeoSteerConfiguration(
                this.motorCANId,
                this.absoluteEncoder,
                nominalVoltage,
                this.currentLimit,
                this.proportionalConstant,
                this.integralConstant,
                this.derivativeConstant
        );
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public NeoSteerConfiguration withCurrentLimit(double currentLimit) {
        return new NeoSteerConfiguration(
                this.motorCANId,
                this.absoluteEncoder,
                this.nominalVoltage,
                currentLimit,
                this.proportionalConstant,
                this.integralConstant,
                this.derivativeConstant
        );
    }

    public NeoSteerConfiguration withPidConstants(double proportional, double integral, double derivative) {
        return new NeoSteerConfiguration(
                this.motorCANId,
                this.absoluteEncoder,
                this.nominalVoltage,
                this.currentLimit,
                proportional,
                integral,
                derivative
        );
    }

    public boolean hasPidConstants() {
        return Double.isFinite(proportionalConstant) && Double.isFinite(integralConstant) && Double.isFinite(derivativeConstant);
    }
}
