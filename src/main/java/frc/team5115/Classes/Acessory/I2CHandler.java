package frc.team5115.Classes.Acessory;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Hardware.NAVx;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import java.util.Arrays;

public class I2CHandler extends SubsystemBase {
    I2C i2c;
    byte[] buffer;
    static final byte rollAddress = 0x1C;
    static final byte pitchAddress = 0x1E;
    static final byte yawAddress = 0x1A;

    static final byte xGravAddress = 0x2E;
    static final byte yGravAddress = 0x30;
    static final byte zGravAddress = 0x32;
    short lastPitch;
    short lastRoll;
    short lastYaw;
    short lastXgrav;
    short lastYgrav;
    short lastZgrav;

    public I2CHandler() {
        i2c = new I2C(Port.kMXP, 0x28);
        i2c.write(0x3D, 12);
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

    public static double[] vectorNormalize(double[] v) {
        double[] result = new double[v.length];
        double sum = 0;
        for (double d : v) {
            sum += d;
        }
        for (int i = 0; i < v.length; i++) {
            result[i] = v[i] / sum;
        }
        return result;
    }

    private double[] getGravity() {
        lastXgrav = readFromSensor(xGravAddress, 2, lastXgrav);        
        lastYgrav = readFromSensor(yGravAddress, 2, lastYgrav);        
        lastZgrav = readFromSensor(zGravAddress, 2, lastZgrav);   
        return vectorNormalize(new double[] {lastXgrav, lastYgrav, lastZgrav});
    }

    private short getPitch() {
        // right now it looks like the yawAddress is actually pitch
        lastPitch = readFromSensor(pitchAddress, 2, lastPitch);
        return lastPitch;
    }

    private short getRoll() {
        lastRoll = readFromSensor(rollAddress, 2, lastRoll);
        return lastRoll;
    }

    private short getYaw() {
        lastYaw = readFromSensor(yawAddress, 2, lastYaw);
        return lastYaw;
    }

    public double getPitchReal() {
        System.out.println("Gravity: " + Arrays.toString(getGravity()));
        return NAVx.clampAngle((double) getYaw() / 16.0 - 96.0);
    }

    private short readFromSensor(byte registerAddress, int count, short defaultValue) {
        final boolean aborted = i2c.read(registerAddress, count, buffer);

        if (aborted) {
            System.out.println("Failed to read from BNO055");
            return defaultValue;
        }
        return combineBytes(buffer); 
    }

    public void Disable() {
        i2c.close();
    }
    
}
