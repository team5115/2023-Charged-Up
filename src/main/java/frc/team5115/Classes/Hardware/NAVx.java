package frc.team5115.Classes.Hardware;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * The NAVx subsystem. Provides methods to interact with the NAVx.
 */
public class NAVx implements Subsystem {

    private final AHRS ahrs = new AHRS();
    private double yawAtReset = 0;
    private double pitchAtReset = 0;
    
	/**
	 * `NAVx` constructor.
	 */
    public NAVx() {
        checkForConnection();
        ahrs.reset();
    }

	/**
	 * Sets the baseline yaw to the current yaw.
	 */
    public void resetYaw(){
        yawAtReset = getYawDeg();
    }

	/**
	 * Sets the baseline pitch to the current pitch.
	 */
    public void resetPitch() {
        pitchAtReset = getPitchDeg();
    }

	/**
	 * Sets the baseline yaw and pitch to the current yaw and pitch.
	 */
    public void resetNAVx() {
        resetYaw();
        resetPitch();
        checkForConnection();
    }

    /**
     * @return The yaw of the navx from the last reset, ranging from -180 to 180 degrees.
     */
    public double getYawDeg() {
        return clampAngle(ahrs.getYaw() - yawAtReset);
    }

    /**
     * @return The pitch of the navx from the last reset, ranging from -180 to 180 degrees.
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
     * Converts an angle into an equivalent angle in the range -180 to 180.
     * @param angle - The input angle to convert
     * @return The equivalent angle from -180 to 180
     */
    public static double clampAngle(double angle) {
        return ((angle + 180.0) % 360.0) - 180.0;
    }

	/**
	 * @return The yaw of the navx from the last reset, ranging from -pi to pi radians
	 */
    public double getYawRad() {
        return Units.degreesToRadians(getYawDeg());
    }

	/**
	 * @return The pitch of the navx from the last reset, ranging from -pi to pi radians
	 */
	public double getPitchRad() {
		return Units.degreesToRadians(getPitchDeg());
	}

	/**
	 * @return The pitch in `Rotation2d` form
	 */
	public Rotation2d getPitchRotation2D() {
		return Rotation2d.fromDegrees(getPitchDeg());
	}

	/**
	 * @return The yaw in `Rotation2d` form
	 */
    public Rotation2d getYawRotation2D() {
        return Rotation2d.fromDegrees(getYawDeg());
    }

	/**
	 * Checks to see if the navx is connected. Prints a message to the console with the result.
	 * @return Whether or not the navx is connected
	 */
    public boolean checkForConnection() {
        if(ahrs.isConnected()) {
            System.out.println("NavX is connected");
            return true;
        }
        System.out.println("NavX is NOT connected!!!");
        return false;
    }
}