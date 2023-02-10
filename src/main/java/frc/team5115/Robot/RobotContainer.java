package frc.team5115.Robot;

import static frc.team5115.Constants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team5115.Classes.Software.*;
import frc.team5115.Classes.Hardware.*;
import edu.wpi.first.wpilibj.DigitalOutput;

public class RobotContainer {
    private final Drivetrain drivetrain;
    private final Pneumatic pneum;
    public final Joystick joy = new Joystick(0);
    private final Timer timer;
    private final DigitalOutput x = new DigitalOutput(0);
    private final IntakeMotor intakeMotor;

    public RobotContainer() {
        pneum = new Pneumatic();
        drivetrain = new Drivetrain();
        intakeMotor = new IntakeMotor();
        timer = new Timer();
        timer.reset();
        configureButtonBindings();
    }

    public void configureButtonBindings() {
    }

    public void startTeleop(){
        x.set(true);
        System.out.println("Starting teleop");
    }

    public void disabledInit(){
        x.set(false);
        pneum.close();
    }

    public void stopEverything(){
        drivetrain.stop();
        intakeMotor.stop();
    }

    public void startAuto(){
    }

    public void autoPeriod(){

    }

    public void teleopPeriodic(){
        double forward = -joy.getRawAxis(JOY_Y_AXIS_ID); // negated because Y axis on controller is negated
        double turn = joy.getRawAxis(JOY_Z_AXIS_ID);
        //drivetrain.TankDriveOld(forward, turn);
    }
}
