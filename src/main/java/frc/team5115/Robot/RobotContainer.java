package frc.team5115.Robot;

import static frc.team5115.Constants.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team5115.Classes.Hardware.HardwareIntake;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Classes.Software.PhotonVision;
import frc.team5115.Commands.Auto.AutoCommandGroup;
import frc.team5115.Commands.Auto.DockAuto.DockCommandGroup;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.HighCone;

public class RobotContainer {
    private final Timer timer;
    private final Joystick joy;
    private final PhotonVision photonVision;
    private final Drivetrain drivetrain;
    private final HardwareIntake intake;
    private final Arm arm;
    private final AutoCommandGroup autoCommandGroup;
    private final HighCone highCone;
    private final DockCommandGroup dockSequence;

    public RobotContainer() {
        joy = new Joystick(0);
        photonVision = new PhotonVision();
        drivetrain = new Drivetrain(photonVision);
        intake = new HardwareIntake();
        arm = new Arm();
        highCone = new HighCone(arm);
        autoCommandGroup = new AutoCommandGroup(drivetrain, arm);
        dockSequence = new DockCommandGroup(drivetrain);
        timer = new Timer();
        timer.reset();
        configureButtonBindings();
    }

    public void configureButtonBindings() {
        //new JoystickButton(joy, 1).onTrue(new InstantCommand(drivetrain :: toggleSlowMode));
        //new JoystickButton(joy, 2).onTrue(dockSequence);
        new JoystickButton(joy, 1).onTrue(new InstantCommand(arm :: In));
        new JoystickButton(joy, 2).onTrue(new InstantCommand(arm :: Out));
        //new JoystickButton(joy, 1).whileTrue((highCone)).onFalse( new InstantCommand(arm :: stop));
    }

    public void startTeleop(){
        if(autoCommandGroup != null) autoCommandGroup.cancel();
        // arm.zeroArm();
        //digitalOutput.set(true);
        drivetrain.resetNAVx();
        System.out.println("Starting teleop");
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
       arm.updateController();
    }

    public void teleopPeriodic(){
        //drivetrain.UpdateOdometry();
        arm.updateController();
        double forward = -joy.getRawAxis(JOY_Y_AXIS_ID); // negated because Y axis on controller is negated
        double turn = joy.getRawAxis(JOY_Z_AXIS_ID);
        drivetrain.TankDrive(forward, turn);
    }
}
