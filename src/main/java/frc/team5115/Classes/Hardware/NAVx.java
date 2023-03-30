package frc.team5115.Classes.Hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class NAVx implements Subsystem {

    private final AHRS ahrs = new AHRS();
    private double yawAtReset = 0;
    private double pitchAtReset = 0;
    
    public NAVx() {
        checkForConnection();
        ahrs.reset();
    }

    public void resetYaw(){
        yawAtReset = getYawDeg();
    }
    public void resetPitch() {
        pitchAtReset = getPitchDeg();
    }
    public void resetNAVx() {
        resetYaw();
        resetPitch();
        checkForConnection();
    }

    /**
     * @return the yaw of the navx from the last reset, ranging from -180 to 180 degrees 
     */
    public double getYawDeg() {
        return clampAngle(ahrs.getYaw() - yawAtReset);
    }

    /**
     * Find the pitch of the robot, from -180 to 180 degrees
     * @return the pitch of the robot
     */
    public double getPitchDeg() {
        // uses roll because the navx is on the side of the robot -- navx roll is robot pitch
        // getRoll() is negated because tilting up (i.e. front of robot is high) should return a positive value out of this function
        // for sideways mount
        // double angle = ahrs.getRoll() - pitchAtReset;
        // for flat mount
        return clampAngle(-ahrs.getRoll() - pitchAtReset);
    }

    /**
     * Converts an angle into an equivalent angle in the range -180 to 180
     * @param angle the input angle to convert
     * @return the equivalent angle from -180 to 180
     */
    private static double clampAngle(double angle) {
        return ((angle + 180.0) % 360.0) - 180.0;
    }

    public double getYawRad() {
        return Units.degreesToRadians(getYawDeg());
    }

    public Rotation2d getYawRotation2D() {
        return Rotation2d.fromDegrees(getYawDeg());
    }

    public boolean checkForConnection() {
        if(ahrs.isConnected()) {
            System.out.println("NavX is connected");
            return true;
        }
        System.out.println("NavX is NOT connected!!!");
        return false;
    }
}