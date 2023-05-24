import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import frc.team5115.Classes.Hardware.NAVx;

public class NAVxTest {  
    private static final double ANGLE_TEST_TOLERANCE = 0.001;
    
    @Test
    void testClampAngle_rolling_negative() {
        final double output = NAVx.clampAngle(-365.0);
        final double expected = -5.0;
        Assertions.assertEquals(expected, output, ANGLE_TEST_TOLERANCE);
    }

    @Test
    void testClampAngle_270() {
        final double output = NAVx.clampAngle(270.9);
        final double expected = -89.1;
        Assertions.assertEquals(expected, output, ANGLE_TEST_TOLERANCE);
    }

    @Test
    void testClampAngle_360() {
        final double output = NAVx.clampAngle(360);
        final double expected = 0;
        Assertions.assertEquals(expected, output, ANGLE_TEST_TOLERANCE);
    }
}