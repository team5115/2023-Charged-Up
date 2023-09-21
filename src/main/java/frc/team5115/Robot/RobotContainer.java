package frc.team5115.Robot;

import static frc.team5115.Constants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5115.Classes.Software.*;
import frc.team5115.Classes.Acessory.JoyAxisBoolSupplier;
import frc.team5115.Classes.Hardware.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DigitalOutput;

public class RobotContainer {
    private final Joystick joy;
    private final PhotonVision photonVision;
    private final Drivetrain drivetrain;
    // private final AutoCommandGroup autoCommandGroup;

    public RobotContainer() {
        joy = new Joystick(0);
        photonVision = new PhotonVision();
        drivetrain = new Drivetrain();
        configureButtonBindings();
    }

    public void configureButtonBindings() {
    }

    public void startTeleop(){
        // if(autoCommandGroup != null) autoCommandGroup.cancel();
        System.out.println("Starting teleop");
    }

    public void disabledInit(){
    }

    public void stopEverything(){
        drivetrain.stop();
    }

    public void startAuto(){
        //if(autoCommandGroup != null) autoCommandGroup.schedule();
    }

    public void autoPeriod(){
       //drivetrain.UpdateOdometry();
    }

    public void teleopPeriodic(){
        //drivetrain.UpdateOdometry();
        double forward = -joy.getRawAxis(JOY_Y_AXIS_ID); // negated because Y axis on controller is negated
        double turn = joy.getRawAxis(JOY_Z_AXIS_ID);
        drivetrain.TankDrive(forward, turn);
    }
}
