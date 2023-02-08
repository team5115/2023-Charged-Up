package frc.team5115.Robot;

import static frc.team5115.Constants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Classes.Software.IntakeMotor;
import frc.team5115.Classes.Software.PhotonVision;
import frc.team5115.Commands.Auto.AutoCommandGroup;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
    private final Drivetrain drivetrain;
    private final PhotonVision photonVision;
    public final Joystick joy = new Joystick(0);
    private final Timer timer;
    private final IntakeMotor intakeMotor;
    private final AutoCommandGroup autoCommandGroup;

    public RobotContainer() {
        photonVision = new PhotonVision();
        drivetrain = new Drivetrain(photonVision);
        intakeMotor = new IntakeMotor();
        autoCommandGroup = new AutoCommandGroup(drivetrain, intakeMotor);
        timer = new Timer();
        timer.reset();
        configureButtonBindings();
    }

    public void configureButtonBindings() {
        new JoystickButton(joy, 1).onTrue(new HighCone(intakeMotor));
        //new JoystickButton( joy, 3).whileTrue(new InstantCommand(intakeMotor :: stop)).onFalse(new InstantCommand(intakeMotor :: stop));
        //new JoystickButton( joy, 4).whileTrue(new InstantCommand(intakeMotor :: stop)).onFalse(new InstantCommand(intakeMotor :: stop));
    }

    public void startTeleop(){
        if(autoCommandGroup != null) autoCommandGroup.cancel();
        drivetrain.resetNAVx();
        System.out.println("Starting teleop");
    }

    public void stopEverything(){
        drivetrain.stop();
        intakeMotor.stop();
    }

    public void startAuto(){
        if(autoCommandGroup != null) autoCommandGroup.schedule();
    }

    public void autoPeriod(){
       drivetrain.UpdateOdometry();
       //intakeMotor.updateController();
    }

    public void teleopPeriodic(){
        //intakeMotor.updateController();
        drivetrain.UpdateOdometry();
        double forward = -joy.getRawAxis(JOY_Y_AXIS_ID); // negated because Y axis on controller is negated
        double turn = joy.getRawAxis(JOY_Z_AXIS_ID);
        drivetrain.TankDriveOld(forward, turn);
    }
}
