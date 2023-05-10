package frc.team5115.Classes.Acessory;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class I2CHandler extends SubsystemBase {
    I2C i2c;
    byte[] buffer;
    static final byte rollAddress = 0x1C;
    static final byte pitchAddress = 0x1E;
    static final byte yawAddress = 0x1A;
    short lastPitch;
    short lastRoll;
    short lastYaw;

    public I2CHandler() {
        i2c = new I2C(Port.kMXP, 0x28);
        i2c.write(0x3D, 12);
        buffer = new byte[2];
    }

    public static short combineBytes(byte msb, byte lsb) {
        return (short)((msb << 8) | (lsb & 0xFF));
    }

    public static short combineBytes(byte[] bytes) {
        if (bytes.length == 1) {
            return (short) bytes[0];
        }
        return combineBytes(bytes[1], bytes[0]);
    }

    public short getPitch() {
        // right now it looks like the yawAddress is actually pitch
        lastPitch = readFromSensor(pitchAddress, 2, lastPitch);
        return lastPitch;
    }

    public short getRoll() {
        lastRoll = readFromSensor(rollAddress, 2, lastRoll);
        return lastRoll;
    }

    public short getYaw() {
        lastYaw = readFromSensor(yawAddress, 2, lastYaw);
        return lastYaw;
    }

    private short readFromSensor(byte registerAddress, int count, short defaultValue) {
        final boolean aborted = i2c.read(registerAddress, count, buffer);

        if (aborted) {
            System.out.println("Failed to read roll from BNO055");
            return defaultValue;
        }
        return combineBytes(buffer);
    }

    public void Disable() {
        i2c.close();
    }
    
}
