package com.swervedrivespecialties.swervelib.ctre;

import com.swervedrivespecialties.swervelib.AbsoluteEncoder;

public class Falcon500SteerConfiguration {
    private static final double DEFAULT_NOMINAL_VOLTAGE = 12;
    private static final double DEFAULT_CURRENT_LIMIT = 20;

    public final int motorCANId;
    public final AbsoluteEncoder absoluteEncoder;
    public final double nominalVoltage;
    public final double currentLimit;
    public final double proportionalConstant;
    public final double integralConstant;
    public final double derivativeConstant;
    public final double velocityConstant;
    public final double accelerationConstant;
    public final double staticConstant;

    public Falcon500SteerConfiguration(
        int motorCANId,
        AbsoluteEncoder absoluteEncoder,
        double nominalVoltage,
        double currentLimit,
        double proportionalConstant,
        double integralConstant,
        double derivativeConstant,
        double velocityConstant,
        double accelerationConstant,
        double staticConstant) {
        this.motorCANId = motorCANId;
        this.absoluteEncoder = absoluteEncoder;
        this.nominalVoltage = nominalVoltage;
        this.currentLimit = currentLimit;
        this.proportionalConstant = proportionalConstant;
        this.integralConstant = integralConstant;
        this.derivativeConstant = derivativeConstant;
        this.velocityConstant = velocityConstant;
        this.accelerationConstant = accelerationConstant;
        this.staticConstant = staticConstant;
    }

    public Falcon500SteerConfiguration(int motorCANId, AbsoluteEncoder absoluteEncoder) {
        this(
            motorCANId,
            absoluteEncoder,
            DEFAULT_NOMINAL_VOLTAGE,
            DEFAULT_CURRENT_LIMIT,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN
        );
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public Falcon500SteerConfiguration withVoltageCompensation(double nominalVoltage) {
        return new Falcon500SteerConfiguration(
                this.motorCANId,
                this.absoluteEncoder,
                nominalVoltage,
                this.currentLimit,
                this.proportionalConstant,
                this.integralConstant,
                this.derivativeConstant,
                this.velocityConstant,
                this.accelerationConstant,
                this.staticConstant
        );
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public Falcon500SteerConfiguration withCurrentLimit(double currentLimit) {
        return new Falcon500SteerConfiguration(
                this.motorCANId,
                this.absoluteEncoder,
                this.nominalVoltage,
                currentLimit,
                this.proportionalConstant,
                this.integralConstant,
                this.derivativeConstant,
                this.velocityConstant,
                this.accelerationConstant,
                this.staticConstant
        );
    }

    public Falcon500SteerConfiguration withPidConstants(double proportional, double integral, double derivative) {
        return new Falcon500SteerConfiguration(
                this.motorCANId,
                this.absoluteEncoder,
                this.nominalVoltage,
                this.currentLimit,
                proportional,
                integral,
                derivative,
                this.velocityConstant,
                this.accelerationConstant,
                this.staticConstant
        );
    }

    public boolean hasPidConstants() {
        return Double.isFinite(proportionalConstant) && Double.isFinite(integralConstant) && Double.isFinite(derivativeConstant);
    }

    public Falcon500SteerConfiguration withMotionMagic(double velocityConstant, double accelerationConstant, double staticConstant) {
        return new Falcon500SteerConfiguration(
                this.motorCANId,
                this.absoluteEncoder,
                this.nominalVoltage,
                this.currentLimit,
                this.proportionalConstant,
                this.integralConstant,
                this.derivativeConstant,
                velocityConstant,
                accelerationConstant,
                staticConstant
        );
    }

    public boolean hasMotionMagic() {
        return Double.isFinite(velocityConstant) && Double.isFinite(accelerationConstant) && Double.isFinite(staticConstant);
    }
}
