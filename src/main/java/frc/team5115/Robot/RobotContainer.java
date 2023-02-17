package frc.team5115.Robot;

import static frc.team5115.Constants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team5115.Classes.Software.*;
import frc.team5115.Classes.Hardware.*;
import frc.team5115.Commands.Auto.AutoCommandGroup;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DigitalOutput;

public class RobotContainer {
    private final Drivetrain drivetrain;
    private final PhotonVision photonVision;
    private final HardwareIntake intake;
    public final Joystick joy = new Joystick(0);
    private final Timer timer;
    private final Arm arm;
    private final HighCone highCone;
    private final AutoCommandGroup autoCommandGroup;
    private DigitalOutput digitalOutput = new DigitalOutput(0);
    private DigitalOutput digitalOutput2 = new DigitalOutput(1);

    public RobotContainer() {
        intake = new HardwareIntake();
        photonVision = new PhotonVision();
        drivetrain = new Drivetrain(photonVision);
        arm = new Arm();
        highCone = new HighCone(arm);
        autoCommandGroup = new AutoCommandGroup(drivetrain, arm);
        timer = new Timer();
        timer.reset();
        configureButtonBindings();
    }

    public void configureButtonBindings() {
       new JoystickButton(joy, 1).onTrue(new InstantCommand(arm :: In));
       new JoystickButton(joy, 2).onTrue(new InstantCommand(arm :: Out));

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

    public void startTeleop(){
        if(autoCommandGroup != null) autoCommandGroup.cancel();
        //drivetrain.resetNAVx();
        //digitalOutput.set(true);
        System.out.println("Starting teleop");
    }

    public void disabledInit(){
        //digitalOutput.set(false);
    }

    public void stopEverything(){
        drivetrain.stop();
        arm.stop();
    }

    public void startAuto(){
        //if(autoCommandGroup != null) autoCommandGroup.schedule();
    }

    public void autoPeriod(){
       //drivetrain.UpdateOdometry();
       arm.updateController();
    }

    public void teleopPeriodic(){
        //drivetrain.UpdateOdometry();
        arm.updateController();
        double forward = -joy.getRawAxis(JOY_Y_AXIS_ID); // negated because Y axis on controller is negated
        double turn = joy.getRawAxis(JOY_Z_AXIS_ID);
        //drivetrain.TankDriveOld(forward, turn);
    }
}
