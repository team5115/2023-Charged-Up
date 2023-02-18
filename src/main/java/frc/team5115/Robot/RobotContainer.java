package frc.team5115.Robot;

import static frc.team5115.Constants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5115.Classes.Software.*;
import frc.team5115.Classes.Acessory.JoyAxisBoolSupplier;
import frc.team5115.Classes.Hardware.*;
import frc.team5115.Commands.Auto.AutoCommandGroup;
import frc.team5115.Commands.Auto.DockAuto.DockCommandGroup;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DigitalOutput;

public class RobotContainer {
    private final Joystick joy;
    private final PhotonVision photonVision;
    private final Drivetrain drivetrain;
    private final DockCommandGroup dockSequence;
    // private final HardwareIntake intake;
    // private final Timer timer;
    // private final Arm arm;
    // private final HighCone highCone;
    // private final AutoCommandGroup autoCommandGroup;
    private boolean drivingEnabled;

    public RobotContainer() {
        joy = new Joystick(0);
        photonVision = new PhotonVision();
        drivetrain = new Drivetrain(photonVision);
        dockSequence = new DockCommandGroup(drivetrain);
        // intake = new HardwareIntake();
        // arm = new Arm();
        // highCone = new HighCone(arm);
        // autoCommandGroup = new AutoCommandGroup(drivetrain, arm);
        // timer = new Timer();
        // timer.reset();
        configureButtonBindings();
    }

    public void configureButtonBindings() {
        new JoystickButton(joy, 1).onTrue(new InstantCommand(drivetrain :: toggleSlowMode));
        new JoystickButton(joy, 2).onTrue(dockSequence);
       //new JoystickButton(joy, 1).whileTrue((highCone)).onFalse( new InstantCommand(arm :: stop));
       // new JoystickButton(joy, 1).whileTrue(new InstantCommand(arm :: setTopWinchSpeed)).onFalse( new InstantCommand(arm :: stop));
       // new JoystickButton(joy, 2).whileTrue(new InstantCommand(arm :: setNegTopWinchSpeed)).onFalse( new InstantCommand(arm :: stop));
       // new JoystickButton(joy, 1).whileTrue(new InstantCommand(arm :: setBottomWinchSpeed)).onFalse( new InstantCommand(arm :: stop));
       // new JoystickButton(joy, 2).whileTrue(new InstantCommand(arm :: setNegBottomWinchSpeed)).onFalse( new InstantCommand(arm :: stop));

        //new JoystickButton(joy, 1).onTrue(HighCone);
        //new JoystickButton(joy, 1).onTrue(new InstantCommand(pneum :: open));
        //new JoystickButton(joy, 2).onTrue(new InstantCommand(pneum :: close));
        //new JoystickButton( joy, 3).whileTrue(new InstantCommand(intakeMotor :: stop)).onFalse(new InstantCommand(intakeMotor :: stop));
        //new JoystickButton( joy, 4).whileTrue(new InstantCommand(intakeMotor :: stop)).onFalse(new InstantCommand(intakeMotor :: stop));
    }

    public void enableDriving() {
        drivingEnabled = true;
    }
    public void disableDriving() {
        drivingEnabled = false;
    }

    public void startTeleop(){
        // if(autoCommandGroup != null) autoCommandGroup.cancel();
        // arm.zeroArm();
        drivetrain.resetNAVx();
        System.out.println("Starting teleop");
        enableDriving();
    }

    public void disabledInit(){
    }

    public void stopEverything(){
        drivetrain.stop();
        // arm.stop();
    }

    public void startAuto(){
        //if(autoCommandGroup != null) autoCommandGroup.schedule();
    }

    public void autoPeriod(){
       //drivetrain.UpdateOdometry();
       //arm.updateController();
    }

    public void teleopPeriodic(){
        //drivetrain.UpdateOdometry();
        //arm.updateController();
        double forward = -joy.getRawAxis(JOY_Y_AXIS_ID); // negated because Y axis on controller is negated
        double turn = joy.getRawAxis(JOY_Z_AXIS_ID);
        drivetrain.TankDrive(forward, turn);
        System.out.println(drivetrain.getLeftDistance());
    }
}
