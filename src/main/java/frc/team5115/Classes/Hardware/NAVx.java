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
        double angle = ahrs.getYaw() - yawAtReset;
        if (angle > 180) {
            return -360 + angle;
        }
        return angle;
    }

    /**
     * Find the pitch of the robot, from -180 to 180 degrees
     * @return the pitch of the robot
     */
    public double getPitchDeg() {
        // uses roll because the navx is on the side of the robot -- navx roll is robot pitch
        // getRoll() is negated because tilting up (i.e. front of robot is high) should return a positive value out of this function
        double angle = ahrs.getRoll() - pitchAtReset;
        if (angle > 180) {
            return -360 + angle;
        }
        return angle;
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