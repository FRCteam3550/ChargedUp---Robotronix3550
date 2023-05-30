package frc.robot.utils;

public class JoystickInput{

    private final double m_deadband;
    private final double m_rangeLimit;
    private final boolean m_isSquared;

        public JoystickInput(double deadband, double rangeLimit, boolean isSquared){
            m_deadband = deadband;
            m_rangeLimit = rangeLimit;
            m_isSquared = isSquared;

        }

        private double deadband(double joystickPosition) {
            if (Math.abs(joystickPosition) > m_deadband) {
                return joystickPosition;
            }
            return 0;
        }

        public double modifyAxis(double joystickPosition) {
            joystickPosition = deadband(joystickPosition);

            if(m_isSquared == true){
                joystickPosition = Math.copySign(joystickPosition * joystickPosition, joystickPosition);
            }

            joystickPosition = joystickPosition * m_rangeLimit;

            return joystickPosition;
        }

}
