package frc.team5115.Classes.Hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class NAVx implements Subsystem {

    private final AHRS ahrs = new AHRS();
    private double angleAtReset = 0;
    
    public NAVx() {
        checkForConnection();
        ahrs.reset();
    }

    public void resetNAVX(){
        angleAtReset = getNavxRotationDeg();
        checkForConnection();
    }

    /**
     * @return the yaw of the navx from the last reset, ranging from -180 to 180 degrees 
     */
    public double getNavxRotationDeg(){
        double angle = ahrs.getYaw() - angleAtReset;
        if(angle > 180) {
            return -360 + angle;
        }
        return angle;
    }

    public double getNavxRotationRad() {
        return Units.degreesToRadians(getNavxRotationDeg());
    }

    public Rotation2d getRotation2D() {
        return Rotation2d.fromDegrees(getNavxRotationDeg());
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