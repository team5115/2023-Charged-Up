package frc.team5115.Classes.Software;

import static frc.team5115.Constants.*;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Acessory.ThrottleControl;
import frc.team5115.Classes.Hardware.HardwareDrivetrain;
import frc.team5115.Classes.Hardware.NAVx;
import edu.wpi.first.math.VecBuilder;

public class Drivetrain extends SubsystemBase{
    
    public NetworkTable ShooterCam;
    public NetworkTableEntry ty;
    public NetworkTableEntry tx;
    public NetworkTableEntry tv;
    private final ThrottleControl throttle;
    private final HardwareDrivetrain drivetrain;
    private double leftSpeed;
    private double rightSpeed;

    public static final double kD = 0.25;
    public static final double hD = 0.044;
    public static final double bA = 10;
    public static final double MaxArea = 0.1;

    public Drivetrain() {
        throttle = new ThrottleControl(1.5, -1.5, 0.2);
        
        drivetrain = new HardwareDrivetrain();
    }

    public void stop() {
        drivetrain.plugandFFDrive(0, 0);
    }

    public double getLeftDistance(){
        return drivetrain.getEncoder(1).getPosition()*NEO_ENCODER_CALIBRATION;
    }

    public double getRightDistance(){
        return drivetrain.getEncoder(2).getPosition()*NEO_ENCODER_CALIBRATION;
    }

    public void resetEncoders() {
        drivetrain.resetEncoders();
    }

    /**
     * Drive the robot using a tankdrive setup.
     * @param forward is for driving forward/backward: positive is forward, negative is backward
     * @param turn is for turning right/left: positive is right, negative is left
     */
    public void TankDrive(double forward, double turn) { 
        leftSpeed = 0;
        rightSpeed = 0;

        drivetrain.plugandFFDrive(leftSpeed, rightSpeed);
    }



    /**
     * Drive forward at speed m/s
     * @param speed
     */
    public void autoDrive(double speed){
        drivetrain.plugandFFDrive(0, 0);
    }
}