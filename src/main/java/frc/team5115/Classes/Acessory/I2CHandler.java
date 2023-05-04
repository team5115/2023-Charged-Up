package frc.team5115.Classes.Acessory;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class I2CHandler extends SubsystemBase {
    I2C i2c;
    byte[] buffer;
    static final byte roll = 0x1C;
    static final int buffersize = 2;
    short lastRoll;

    public I2CHandler() {
        i2c = new I2C(Port.kMXP, 0x28);
        i2c.write(0x3D, 12);
        buffer = new byte[buffersize];
    }

    public static short combineBytes(byte msb, byte lsb) {
        return (short)((msb << 8) | (lsb & 0xFF));
    }

    public static short combineBytes(byte[] bytes) {
        return combineBytes(bytes[1], bytes[0]);
    }

    public short getPitch() {

        boolean aborted = i2c.read(roll, buffersize, buffer);

        if(aborted)
            System.out.println("Failed to read roll from BNO055");
        else
            lastRoll = combineBytes(buffer);

        return lastRoll;
    }

    public void Disable() {
        i2c.close();
    }
    
}
