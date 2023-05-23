package frc.team5115.Classes.Acessory;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Hardware.NAVx;
import frc.team5115.Classes.Hardware.NAVx;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import java.util.Arrays;
import java.util.Locale;
import java.util.Locale;

public class I2CHandler extends SubsystemBase {
    I2C i2c;
    byte[] buffer;

    static final byte OPERATION_MODE = 12;
    static final byte OPERATION_MODE_ADDRESS = 0x3D;
    static final byte rollAddress = 0x1C;
    static final byte pitchAddress = 0x1E;
    static final byte yawAddress = 0x1A;

    static final byte xGravAddress = 0x2E;
    static final byte yGravAddress = 0x30;
    static final byte zGravAddress = 0x32;
    short gravX;
    short gravY;
    short gravZ;

    public I2CHandler() {
        i2c = new I2C(Port.kMXP, 0x28);
        i2c.write(OPERATION_MODE_ADDRESS, OPERATION_MODE);
        buffer = new byte[2];
    }

    public static short combineBytes(byte lsb, byte msb) {
        return (short)((msb << 8) | (lsb & 0xFF));
    }

    public static short combineBytes(byte[] bytes) {
        if (bytes.length == 1) {
            return (short) bytes[0];
        }
        return combineBytes(bytes[0], bytes[1]);
    }

    private double[] getGravity() {
        gravX = readFromSensor(xGravAddress, 2, gravX);
        gravY = readFromSensor(yGravAddress, 2, gravY);
        gravZ = readFromSensor(zGravAddress, 2, gravZ);
        return new double[] {gravX/100.0, gravY/100.0, gravZ/100.0};
    }

    public double getPitch() {
        double[] gravity = getGravity();
        final double angle = Math.atan2(-gravity[1], -gravity[0]);
        return Math.toDegrees(angle);

    }

    private short readFromSensor(byte registerAddress, int count, short defaultValue) {
        final boolean aborted = i2c.read(registerAddress, count, buffer);

        if (aborted) {
            System.out.println("Failed to read from BNO055");
            System.out.println("Failed to read from BNO055");
            return defaultValue;
        }
        return combineBytes(buffer);
    }

    public void Disable() {
        i2c.close();
    }
    
}
