package frc.team5115.Classes.Software;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Hardware.HardwareDrivetrain;

public class Drivetrain extends SubsystemBase{
    
    private final HardwareDrivetrain drivetrain;
    private double leftSpeed;
    private double rightSpeed;

    public Drivetrain() {
        drivetrain = new HardwareDrivetrain();
    }

    public void stop() {
        drivetrain.plugandFFDrive(0, 0);
    }

    public void resetEncoders() {
        drivetrain.resetEncoders();
    }

    @Deprecated
    public void TankDriveOld(double forward, double turn){
        leftSpeed = (forward + turn);
        rightSpeed = (forward - turn);
        drivetrain.plugandChugDrive(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
    }

    /**
     * Drive the robot using a tankdrive setup.
     * @param forward is for driving forward/backward: positive is forward, negative is backward
     * @param turn is for turning right/left: positive is right, negative is left
     */
    public void TankDrive(double forward, double turn) { 

        leftSpeed = (forward + turn);
        rightSpeed = (forward - turn);

        leftSpeed *= 3;
        rightSpeed *= 3;

        drivetrain.plugandFFDrive(leftSpeed, rightSpeed);
    }

    /**
     * Drive forward at 1 m/s
     */
    public void autoDriveForward(){
        drivetrain.plugandFFDrive(1, 1);
    }

    @Deprecated
    public void autoDriveF(){
        drivetrain.plugandChugDrive(0.3, -0.3, 0.3, -0.3);
    }

    @Deprecated
    public void autoDriveB(){
        drivetrain.plugandChugDrive(-0.3, 0.3, -0.3, 0.3);
    }
    /**
     * Drive backward at 1 m/s
     */
    public void autoDriveBackward(){
        drivetrain.plugandFFDrive(-1, -1);
    }
}