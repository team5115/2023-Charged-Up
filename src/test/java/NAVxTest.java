import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import frc.team5115.Classes.Hardware.NAVx;

public class NAVxTest {   
    @Test
    void testClampAngle() {
        final double output = NAVx.clampAngle(-365);
        final double expected = -5;
        Assertions.assertEquals(expected, output);
    }
}