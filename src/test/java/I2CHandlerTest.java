import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import frc.team5115.Classes.Acessory.I2CHandler;

public class I2CHandlerTest {   
    @Test
    void testCombineBytes_zeros() {
        final short combined = I2CHandler.combineBytes((byte) 0, (byte) 0);
        final short expected = 0;
        Assertions.assertEquals(expected, combined);
    }

    @Test
    void testCombineBytes_oneeight() {
        final short combined = I2CHandler.combineBytes((byte) 1, (byte) 8);
        final short expected = 264;
        Assertions.assertEquals(expected, combined);
    }
    
    @Test
    void testCombineBytes_ninenine() {
        final short combined = I2CHandler.combineBytes((byte) 9, (byte) 9);
        final short expected = 2313;
        Assertions.assertEquals(expected, combined);
    }
}