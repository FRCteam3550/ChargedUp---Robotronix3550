package com.swervedrivespecialities.swervelib;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import com.swervedrivespecialties.swervelib.DiscreetAngle;


public class DiscreetAngleTest {
    private static double TOLERANCE = 0.00000000001;
    @Test
    public void whenGivingDegreesThenRadiansOk() {
        assertEquals(0, DiscreetAngle.fromDegrees(0).radians());
        assertEquals(Math.PI / 2, DiscreetAngle.fromDegrees(90).radians());
        assertEquals(Math.PI, DiscreetAngle.fromDegrees(180).radians());
    }    

    @Test
    public void whenGivingRadiansThenDegreesOk() {
        assertEquals(0, DiscreetAngle.fromRadians(0).degrees());
        assertEquals(90, DiscreetAngle.fromRadians(Math.PI / 2).degrees());
        assertEquals(180, DiscreetAngle.fromRadians(Math.PI).degrees());
    }    

    @Test
    public void whenGivingDegreesThenDegreesUnchanged() {
        assertEquals(0, DiscreetAngle.fromDegrees(0).degrees());
        assertEquals(90, DiscreetAngle.fromDegrees(90).degrees());
        assertEquals(180, DiscreetAngle.fromDegrees(180).degrees());
    }    

    @Test
    public void whenGivingRadiansThenRadiansUnchanged() {
        assertEquals(0, DiscreetAngle.fromRadians(0).radians());
        assertEquals(Math.PI / 2, DiscreetAngle.fromRadians(Math.PI / 2).radians());
        assertEquals(Math.PI, DiscreetAngle.fromRadians(Math.PI).radians());
    } 
        
    @Test
    public void whenGivingTooHighThenDialedBackWithin360Degrees() {
        assertEquals(0, DiscreetAngle.fromDegrees(360).degrees());
        assertEquals(40, DiscreetAngle.fromDegrees(400).degrees());
        assertEquals(80, DiscreetAngle.fromDegrees(800).degrees());
    } 
        
    @Test
    public void whenGivingTooLowThenDialedBackWithin360Degrees() {
        assertEquals(320, DiscreetAngle.fromDegrees(-40).degrees());
        assertEquals(0, DiscreetAngle.fromDegrees(-360).degrees(), TOLERANCE);
        assertEquals(320, DiscreetAngle.fromDegrees(-400).degrees());
    } 
}
