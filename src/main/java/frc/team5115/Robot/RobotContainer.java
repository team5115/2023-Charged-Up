package frc.team5115.Robot;

import static frc.team5115.Constants.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5115.Classes.Acessory.JoyAxisBoolSupplier;
import frc.team5115.Classes.Hardware.HardwareArm;
import frc.team5115.Classes.Hardware.HardwareIntake;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Classes.Software.PhotonVision;
import frc.team5115.Commands.Auto.AutoCommandGroup;
import frc.team5115.Commands.Auto.DockAuto.DockCommandGroup;
import frc.team5115.Commands.Intake.RealExtend;
import frc.team5115.Commands.Intake.Startup;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.HighCone;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.HighCube;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.MiddleCone;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.MiddleCube;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class RobotContainer {
    private final Timer timer;
    private final Joystick joy1;
    private final Joystick joy2;
    private final PhotonVision photonVision;
    private final Drivetrain drivetrain;
    private final HardwareIntake intake;
    private final Arm arm;
    private final HardwareArm hardwareArm;
    private final AutoCommandGroup autoCommandGroup;
    private final DockCommandGroup dockSequence;
    private final Startup startup;

    // private final HighCone highCone;
    // private final MiddleCone middleCone;
    // private final HighCube highCube;
    // private final MiddleCube middleCube;

    public RobotContainer() {
        joy1 = new Joystick(0);
        joy2 = new Joystick(1);

        photonVision = new PhotonVision();
        drivetrain = new Drivetrain(photonVision);
        intake = new HardwareIntake();
        hardwareArm = new HardwareArm();
        arm = new Arm(hardwareArm);
        
        startup = new Startup(arm, hardwareArm, intake);
        // highCone = new HighCone(arm);
        // middleCone = new MiddleCone(arm);
        // highCube = new HighCube(arm);
        // middleCube = new MiddleCube(arm);
        
        autoCommandGroup = new AutoCommandGroup(drivetrain, arm);
        dockSequence = new DockCommandGroup(drivetrain);
        timer = new Timer();
        timer.reset();
        
        configureButtonBindings();
    }

    public void configureButtonBindings() {
        //new JoystickButton(joy, 1).onTrue(new InstantCommand(drivetrain :: toggleSlowMode));
        //new JoystickButton(joy, 2).onTrue(dockSequence);
        //new JoystickButton(joy1, 1).onTrue(new InstantCommand(arm :: In));
        //new JoystickButton(joy1, 2).onTrue(new InstantCommand(arm :: Out));
        new JoystickButton(joy1, 1).onTrue(new RealExtend(arm, 0));
        new JoystickButton(joy1, 2).onTrue(new RealExtend(arm, 26));
        new JoystickButton(joy1, 3).onTrue(new InstantCommand(arm :: setArmUp));
        new JoystickButton(joy1, 4).onTrue(new InstantCommand(arm :: setArmDown));
        new JoystickButton(joy1, 5).onTrue(new InstantCommand(intake :: TurnOut)).onFalse(new InstantCommand(intake :: StopMotor));
        new JoystickButton(joy1, 6).onTrue(new InstantCommand(intake :: TurnIn)).onFalse(new InstantCommand(intake :: StopMotor));
        new JoystickButton(joy1, 7).onTrue(new InstantCommand(arm :: moveTop));
        new JoystickButton(joy1, 8).onTrue(new InstantCommand(arm :: moveBottom));
        /* 
        new JoystickButton(joy2, 1).onTrue(new InstantCommand(arm :: In));
        new JoystickButton(joy2, 2).onTrue(new InstantCommand(arm :: Out));
        new JoystickButton(joy2, 3).onTrue(new InstantCommand(arm :: setArmUp));
        new JoystickButton(joy2, 4).onTrue(new InstantCommand(arm :: setArmDown));
        new JoystickButton(joy2, 5).onTrue(new InstantCommand(intake :: TurnOut)).onFalse(new InstantCommand(intake :: StopMotor));
        new JoystickButton(joy2, 6).onTrue(new InstantCommand(intake :: TurnIn)).onFalse(new InstantCommand(intake :: StopMotor));
        new JoystickButton(joy2, 7).onTrue(new InstantCommand(arm :: Reset));
        new JoystickButton(joy2, 8).onTrue(new InstantCommand(arm :: setArmStart));

        */
        // BooleanSupplier leftTrigger = new JoyAxisBoolSupplier(joy, 2, 0.5);
        // BooleanSupplier rightTrigger = new JoyAxisBoolSupplier(joy, 3, 0.5);
        // new Trigger(leftTrigger).onTrue((highCube));
        // new JoystickButton(joy, 5).onTrue((middleCube));
        // new Trigger(rightTrigger).onTrue((highCone));
        // new JoystickButton(joy, 6).onTrue((middleCone));
    }

    public void startTeleop(){
        if(autoCommandGroup != null) autoCommandGroup.cancel();
        // arm.zeroArm();
        //digitalOutput.set(true);
        drivetrain.resetNAVx();
        System.out.println("Starting teleop");
        startup.schedule();
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
        if(arm.armcontrol) arm.updateController();
        double forward = -joy1.getRawAxis(JOY_Y_AXIS_ID); // negated because Y axis on controller is negated
        double turn = joy1.getRawAxis(JOY_Z_AXIS_ID);
        //drivetrain.TankDriveOld(forward, turn);
    }
}
