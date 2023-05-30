package frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpeedFilter implements Sendable {
    private final static double DEFAULT_DEADBAND = 0.05;
    private final static double DEFAULT_PRECISE_RANGE_LIMIT = 0.4;

    private final double m_deadbandPct;
    private final double m_fastModeLimitPct;
    private final double m_preciseModeLimitPct;
    private final double m_translationConversionRatio;
    private final double m_rotationConversionRatio;
    private final double m_translationAccelerationLimitPctPerS;
    private final DoubleSupplier m_timer;

    private double m_currentModeLimitPct;
    private double m_previousXSpeedPct = 0;
    private double m_previousYSpeedPct = 0;
    private double m_previousTime = -1;
    private boolean m_squareInputs = false;

    public SpeedFilter() {
        this(DEFAULT_DEADBAND, 1, DEFAULT_PRECISE_RANGE_LIMIT, Double.NaN, 1, 1, createDefaultTimer());
    }

    /**
     * Creates a filter for gamepad inputs for a swerve drivetrain.
     * 
     * @param deadbandPct the input range around 0 where the output will stay at precisely 0. Default to 5%.
     * @param fastModeLimitPct when switching to "fast mode", what's the maximum input. Default to 100%.
     * @param preciseModeLimitPct when switching to "precise mode", what's the maximum input. Default to 40%.
     * @param translationAccelerationLimitPctPerS which maximum rate of change of the input speed we allow for translations. In percent per second. Default to none.
     * @param translationConversionRatio the multiplicative ratio to apply if we want to apply a unit conversion to the percent input.
     * @param rotationConversionRatio the multiplicative ratio to apply if we want to apply a unit conversion to the percent input.
     */
    public SpeedFilter(
        double deadbandPct,
        double fastModeLimitPct,
        double preciseModeLimitPct,
        double translationAccelerationLimitPctPerS,
        double translationConversionRatio,
        double rotationConversionRatio,
        DoubleSupplier timer
    ) {
        m_deadbandPct = deadbandPct;
        m_fastModeLimitPct = fastModeLimitPct;
        m_preciseModeLimitPct = preciseModeLimitPct;
        m_currentModeLimitPct = m_preciseModeLimitPct;
        m_translationAccelerationLimitPctPerS = translationAccelerationLimitPctPerS;
        m_translationConversionRatio = translationConversionRatio;
        m_rotationConversionRatio = rotationConversionRatio;
        m_timer = timer;

        SendableRegistry.add(this, "SpeedFilter");
        SmartDashboard.putData(this);
    }

    private static DoubleSupplier createDefaultTimer() {
        final var timer = new Timer();
        timer.start();
        return timer::get;
    }

    /**
     * Returns a DriveTrainInputFilter with the given dead band and the other parameters unchanged.
     * @param deadbandPct the input range around 0 where the output will stay at precisely 0.
     */
    public SpeedFilter withDeadband(double deadbandPct) {
        return new SpeedFilter(
            deadbandPct,
            m_fastModeLimitPct,
            m_preciseModeLimitPct,
            m_translationAccelerationLimitPctPerS,
            m_translationConversionRatio,
            m_rotationConversionRatio,
            m_timer
        );
    }

    /**
     * Returns a DriveTrainInputFilter with the given range limits and the other parameters unchanged.
     * @param fastModeLimitPct when switching to "fast mode", what's the maximum input.
     * @param preciseModeLimitPct when switching to "precise mode", what's the maximum input.
     */
    public SpeedFilter withRangeLimits(double fastModeLimitPct, double preciseModeLimitPct) {
        return new SpeedFilter(
            m_deadbandPct,
            fastModeLimitPct,
            preciseModeLimitPct,
            m_translationAccelerationLimitPctPerS,
            m_translationConversionRatio,
            m_rotationConversionRatio,
            m_timer
        );
    }

    /**
     * Returns a DriveTrainInputFilter with the given input speed limit and the other parameters unchanged.
     * @param translationInputSpeedLimitPctPerS which maximum rate of change of the input we allow for translations. In percent per second.
     */
    public SpeedFilter withTranslationInputSpeedLimit(double translationInputSpeedLimitPctPerS) {
        return new SpeedFilter(
            m_deadbandPct,
            m_fastModeLimitPct,
            m_preciseModeLimitPct,
            translationInputSpeedLimitPctPerS,
            m_translationConversionRatio,
            m_rotationConversionRatio,
            m_timer
        );
    }   
    
    /**
     * Returns a DriveTrainInputFilter with the given conversion ratios and the other parameters unchanged.
     * @param translationConversionRatio the multiplicative ratio to apply if we want to apply a unit conversion to the percent input.
     * @param rotationConversionRatio the multiplicative ratio to apply if we want to apply a unit conversion to the percent input.
     */
    public SpeedFilter withConversionRatios(double translationConversionRatio, double rotationConversionRatio) {
        return new SpeedFilter(
            m_deadbandPct,
            m_fastModeLimitPct,
            m_preciseModeLimitPct,
            m_translationAccelerationLimitPctPerS,
            translationConversionRatio,
            rotationConversionRatio,
            m_timer
        );
    }

    public void switchToFastMode() {
        m_currentModeLimitPct = m_fastModeLimitPct;
        m_squareInputs = true;
    }

    public void switchToPreciseMode() {
        m_currentModeLimitPct = m_preciseModeLimitPct;
        m_squareInputs = false;
    }

    private static double square(double input) {
        return Math.copySign(input * input, input);
    }

    private double applyDeadband(double input) {
        if (Math.abs(input) <= m_deadbandPct) {
            return 0;
        }
        return input;
    }

    /**
     * Modify the inputs according to the various parameters and current mode (fast or precise).
     * @param inputsPct desired speeds from the gamepad.
     * @return filtered speeds for the drivetrain. Units are percentages by default, but could be altered by using withConversionRatios().
     */
    public SpeedSetPoint filter(SpeedSetPoint inputsPct) {
        var xSpeedPct = applyDeadband(inputsPct.xSpeed) * m_currentModeLimitPct;
        var ySpeedPct = applyDeadband(inputsPct.ySpeed) * m_currentModeLimitPct;
        var rotationSpeedPct = applyDeadband(inputsPct.rotationSpeed) * m_currentModeLimitPct;

        // Do we want to square the inputs?
        if (m_squareInputs) {
            xSpeedPct = square(xSpeedPct);
            ySpeedPct = square(ySpeedPct);
            rotationSpeedPct = square(rotationSpeedPct);
        }

        // Do we want to apply an acceleration limit?
        if (Double.isFinite(m_translationAccelerationLimitPctPerS)){
            var currentTime = m_timer.getAsDouble();
            var durationS = m_previousTime < 0 ? 1 : currentTime - m_previousTime;

            var variationXPctS = xSpeedPct - m_previousXSpeedPct;
            var variationYPctS = ySpeedPct - m_previousYSpeedPct;
            var variation = Math.sqrt(variationXPctS * variationXPctS + variationYPctS * variationYPctS) / durationS;

            // We over accelerate, we need to reduce to the max acceleration
            if (variation > m_translationAccelerationLimitPctPerS) {
                var reductionFactor = m_translationAccelerationLimitPctPerS / variation;
                xSpeedPct = m_previousXSpeedPct + variationXPctS * reductionFactor;
                ySpeedPct = m_previousYSpeedPct + variationYPctS * reductionFactor;
            }

            m_previousTime = currentTime;
            m_previousXSpeedPct = xSpeedPct;
            m_previousYSpeedPct = ySpeedPct;
        }

        return new SpeedSetPoint(
            xSpeedPct * m_translationConversionRatio,
            ySpeedPct * m_translationConversionRatio,
            rotationSpeedPct * m_rotationConversionRatio
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("SpeedLimit", () -> m_currentModeLimitPct, null);
        
    }
}
