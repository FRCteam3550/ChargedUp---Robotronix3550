package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Leds extends SubsystemBase {
    private final Solenoid m_yellowLeftLed = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Leds.LED_YELLOW_LEFT_CHANNEL);
    private final Solenoid m_yellowRightLed = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Leds.LED_YELLOW_RIGHT_CHANNEL);

    private final Solenoid m_greenLeftLed = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Leds.LED_GREEN_LEFT_CHANNEL);
    private final Solenoid m_greenRightLed = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Leds.LED_GREEN_RIGHT_CHANNEL);

    public Command wantCone() {
        return runOnce(() -> {
            m_greenLeftLed.set(false);
            m_greenRightLed.set(false);
            m_yellowLeftLed.set(true);
            m_yellowRightLed.set(true);
        });
    }

    public Command wantCube() {
        return runOnce(() -> {
            m_yellowLeftLed.set(false);
            m_yellowRightLed.set(false);
            m_greenLeftLed.set(true);
            m_greenRightLed.set(true);
        });
    }

    public Command stopAsking() {
        return runOnce(() -> {
            m_yellowLeftLed.set(false);
            m_yellowRightLed.set(false);
            m_greenLeftLed.set(false);
            m_greenRightLed.set(false);

        });
    }
}
