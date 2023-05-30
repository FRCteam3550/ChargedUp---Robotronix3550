package frc.robot;

import com.swervedrivespecialties.swervelib.DiscreetAngle;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double TEMP_MAX_SPEED_MS = 4.786;

    public static final int BRAKE_MODE_LIMITSWITCH_ID = 8;
    
    public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 3;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 7;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER_ID = 9;
    public static final DiscreetAngle FRONT_LEFT_MODULE_STEER_ALIGN_ANGLE = DiscreetAngle.fromDegrees(63.38);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 1;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER_ID = 12;
    public static final DiscreetAngle FRONT_RIGHT_MODULE_STEER_ALIGN_ANGLE = DiscreetAngle.fromDegrees(13.65);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ID = 2;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR_ID = 8;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER_ID = 11;
    public static final DiscreetAngle BACK_LEFT_MODULE_STEER_ALIGN_ANGLE = DiscreetAngle.fromDegrees(352.79);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ID = 6;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR_ID = 4;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER_ID = 0;
    public static final DiscreetAngle BACK_RIGHT_MODULE_STEER_ALIGN_ANGLE = DiscreetAngle.fromDegrees(212.69);

    public static class Elevator {
        public static final int RIGHT_MOTOR_ID = 14;
        public static final int LEFT_MOTOR_ID = 15;

        public static final int BOTTOM_LIMITSWITCH_ID = 0;
        public static final int TOP_LIMITSWITCH_ID = 1;
    }

    public static class Extensor {
        public static final int MOTOR_ID = 16;
        public static final int RETRACTED_LIMITSWITCH_ID = 2;
        public static final int EXTENDED_LIMITSWITCH_ID = 3;    
    }

    public static class Pivot {
        public static final int MOTOR_ID = 17;
        public static final int EXTENDED_LIMITSWITCH_ID = 5;
        public static final int RETRACTED_LIMITSWITCH_ID = 4;
    }

    public static class Claw {
        public static final int MOTOR_ID = 18;
        public static final int FORWARD_PCM_CHANNEL = 6;
        public static final int REVERSE_PCM_CHANNEL = 7;
        public static final int CATCH_PCM_LED_CHANNEL = 4;
        public static final int LIMIT_SWITCH_ID = 7;

    }

    public static class NeoMotor {
        public static final int TICKS_PER_REV = 42;
    }

    public static class Leds {
        public static final int LED_YELLOW_LEFT_CHANNEL = 0;
        public static final int LED_GREEN_LEFT_CHANNEL = 1;
        public static final int LED_YELLOW_RIGHT_CHANNEL = 2;
        
        public static final int LED_GREEN_RIGHT_CHANNEL = 3;

    }
}
