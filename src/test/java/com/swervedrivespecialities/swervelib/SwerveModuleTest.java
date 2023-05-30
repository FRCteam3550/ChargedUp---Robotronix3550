package com.swervedrivespecialities.swervelib;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import com.swervedrivespecialties.swervelib.ContinuousAngle;
import com.swervedrivespecialties.swervelib.DiscreetAngle;
import com.swervedrivespecialties.swervelib.SwerveModule;


public class SwerveModuleTest {
    static final double EPSILON = 0.0000001;

    static void assertSetpointEquals(double expectedAngleDegrees, double expectedDriveSign, SwerveModule.SteerSetPoint actual) {
        assertEquals(expectedAngleDegrees, actual.targetAngle.degrees(), EPSILON);
        assertEquals(expectedDriveSign, actual.driveSign);
    }

    static  SwerveModule.SteerSetPoint getSteerAngleAndDriveSign(double targetAngleDegrees, double currentAngleDegrees) {
        return SwerveModule.getSteerAngleAndDriveSign(
            DiscreetAngle.fromDegrees(targetAngleDegrees),
            ContinuousAngle.fromDegrees(currentAngleDegrees)
        );
    }

    @Test
    void when0CurrentAngle() {
        var currentDegrees = 0;

        assertSetpointEquals(0, 1, getSteerAngleAndDriveSign(0, currentDegrees));
        assertSetpointEquals(90, 1, getSteerAngleAndDriveSign(90, currentDegrees));
        assertSetpointEquals(0, -1, getSteerAngleAndDriveSign(180, currentDegrees));
        assertSetpointEquals(-90, 1, getSteerAngleAndDriveSign(270, currentDegrees));
        assertSetpointEquals(-10, 1, getSteerAngleAndDriveSign(350, currentDegrees));
    }    

    @Test
    void when360CurrentAngle() {
        var currentDegrees = 360;

        assertSetpointEquals(360, 1, getSteerAngleAndDriveSign(0, currentDegrees));
        assertSetpointEquals(450, 1, getSteerAngleAndDriveSign(90, currentDegrees));
        assertSetpointEquals(360, -1, getSteerAngleAndDriveSign(180, currentDegrees));
        assertSetpointEquals(270, 1, getSteerAngleAndDriveSign(270, currentDegrees));
        assertSetpointEquals(350, 1, getSteerAngleAndDriveSign(350, currentDegrees));
    }    

    @Test
    void whenSmallCurrentAngle() {
        var currentDegrees = 10;

        assertSetpointEquals(0, 1, getSteerAngleAndDriveSign(0, currentDegrees));
        assertSetpointEquals(90, 1, getSteerAngleAndDriveSign(90, currentDegrees));
        assertSetpointEquals(0, -1, getSteerAngleAndDriveSign(180, currentDegrees));
        assertSetpointEquals(90, -1, getSteerAngleAndDriveSign(270, currentDegrees));
        assertSetpointEquals(-10, 1, getSteerAngleAndDriveSign(350, currentDegrees));
    }    

    @Test
    void whenLargeCurrentAngle() {
        var currentDegrees = 350;

        assertSetpointEquals(360, 1, getSteerAngleAndDriveSign(0, currentDegrees));
        assertSetpointEquals(270, -1, getSteerAngleAndDriveSign(90, currentDegrees));
        assertSetpointEquals(360, -1, getSteerAngleAndDriveSign(180, currentDegrees));
        assertSetpointEquals(270, 1, getSteerAngleAndDriveSign(270, currentDegrees));
        assertSetpointEquals(350, 1, getSteerAngleAndDriveSign(350, currentDegrees));
    } 

    @Test
    void whenCurrentAngleMoreThan360() {
        var currentDegrees = 710;

        assertSetpointEquals(720, 1, getSteerAngleAndDriveSign(0, currentDegrees));
        assertSetpointEquals(630, -1, getSteerAngleAndDriveSign(90, currentDegrees));
        assertSetpointEquals(720, -1, getSteerAngleAndDriveSign(180, currentDegrees));
        assertSetpointEquals(630, 1, getSteerAngleAndDriveSign(270, currentDegrees));
        assertSetpointEquals(710, 1, getSteerAngleAndDriveSign(350, currentDegrees));
    } 

    @Test
    void whenNegativeAngle() {
        var currentDegrees = -360;

        assertSetpointEquals(-360, 1, getSteerAngleAndDriveSign(0, currentDegrees));
        assertSetpointEquals(-270, 1, getSteerAngleAndDriveSign(90, currentDegrees));
        assertSetpointEquals(-360, -1, getSteerAngleAndDriveSign(180, currentDegrees));
        assertSetpointEquals(-450, 1, getSteerAngleAndDriveSign(270, currentDegrees));
        assertSetpointEquals(-370, 1, getSteerAngleAndDriveSign(350, currentDegrees));
    } 

    @Test
    void whenNegativeAngleTest2() {
        var currentDegrees = -370;

        assertSetpointEquals(-360, 1, getSteerAngleAndDriveSign(0, currentDegrees));
        assertSetpointEquals(-450, -1, getSteerAngleAndDriveSign(90, currentDegrees));
        assertSetpointEquals(-360, -1, getSteerAngleAndDriveSign(180, currentDegrees));
        assertSetpointEquals(-450, 1, getSteerAngleAndDriveSign(270, currentDegrees));
        assertSetpointEquals(-370, 1, getSteerAngleAndDriveSign(350, currentDegrees));
    } 
}
