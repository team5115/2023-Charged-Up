package frc.team5115.Robot;

import static frc.team5115.Constants.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team5115.Classes.Acessory.*;
import frc.team5115.Classes.Hardware.*;
import frc.team5115.Classes.Software.*;
import frc.team5115.Commands.Auto.AutoCommandGroup;
import frc.team5115.Commands.Auto.DockAuto.DockCommandGroup;
import frc.team5115.Commands.Intake.*;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.*;
import frc.team5115.Commands.Intake.RawIntakeCommands.*;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.commands.FollowPathWithEvents;

public class RobotContainer {
    private final Timer timer;
    private final Joystick joy1;
    private final Joystick joy2;
    private final PhotonVision photonVision;
    private final Drivetrain drivetrain;
    private final HardwareIntake intake;
    private final Arm arm;
    private final HardwareArm hardwareArm;
    private FollowPathWithEvents autoCommandGroup;
    private final DockCommandGroup dockSequence;
    private final Startup startup;
    private ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
    private GenericEntry good = tab.add("good auto?", false).getEntry();
    private boolean goodAuto = false; 

    public RobotContainer() {
        joy1 = new Joystick(0);
        joy2 = new Joystick(1);

        photonVision = new PhotonVision();
        intake = new HardwareIntake();
        hardwareArm = new HardwareArm();
        arm = new Arm(hardwareArm, intake);
        drivetrain = new Drivetrain(photonVision, arm);
        startup = new Startup(arm, hardwareArm, intake);
        autoCommandGroup = drivetrain.C;
        dockSequence = new DockCommandGroup(drivetrain, false);
        timer = new Timer();
        timer.reset();
        configureButtonBindings();
    }

    public void configureButtonBindings() {
        new JoystickButton(joy2, 1).onTrue(new InstantCommand(drivetrain :: toggleSlowMode));
        //new JoystickButton(joy2, 9).onTrue(dockSequence);
        //new JoystickButton(joy1, 2).onTrue(new RealExtend(arm, 0));
        //new JoystickButton(joy1, 1).onTrue(new RealExtend(arm, 25.5));
        // new JoystickButton(joy1, 1).onTrue(new IntakeExtend_v2(arm, 0, 0));
        // new JoystickButton(joy1, 2).onTrue(new IntakeExtend_v2(arm, 25.5, 25.5));
        // new JoystickButton(joy1, 3).onTrue(new InstantCommand(arm :: setArmUp));
        // new JoystickButton(joy1, 4).onTrue(new InstantCommand(arm :: setArmDown));
        // new JoystickButton(joy1, 5).onTrue(new InstantCommand(intake :: TurnOut)).onFalse(new InstantCommand(intake :: StopMotor));
        // new JoystickButton(joy1, 6).onTrue(new InstantCommand(intake :: TurnIn)).onFalse(new InstantCommand(intake :: StopMotor));
        // new JoystickButton(joy1, 7).onTrue(new HighCone(arm));
        // new JoystickButton(joy1, 8).onTrue(new Resting(arm));
        
        new JoystickButton(joy1, 3).onTrue(new ShelfSubstation(arm)); // double substation pickup
        new JoystickButton(joy1, 4).onTrue(new HighNode(arm)); // high node
        new JoystickButton(joy1, 2).onTrue(new MiddleNode(arm)); // middle node
        new JoystickButton(joy1, 1).onTrue(new GroundPickup(arm)); // low node/ground pickup
        new JoystickButton(joy1, 8).onTrue(new Stow(arm, hardwareArm, intake)); // stow fully
        new JoystickButton(joy1, 7).onTrue(new StowCone(arm, hardwareArm, intake)); // stow with cone
        
       // new Trigger(new JoyAxisBoolSupplier(joy1, 1, -0.5, false)).onTrue(new InstantCommand(arm :: turnUp)); // angle up
       // new Trigger(new JoyAxisBoolSupplier(joy1, 1, +0.5, true)).onTrue(new InstantCommand(arm :: turnDown)); // angle down
        //new JoystickButton(joy1, 5).whileTrue(new InstantCommand(arm :: topMoveIn)); // top in
       // new JoystickButton(joy1, 6).whileTrue(new InstantCommand(arm :: topMoveOut)); // top out
     //   new Trigger(new JoyAxisBoolSupplier(joy1, 2, +0.5)).onTrue(new InstantCommand(arm :: bottomMoveIn)); // bottom in
      //  new Trigger(new JoyAxisBoolSupplier(joy1, 3, +0.5)).onTrue(new InstantCommand(arm :: bottomMoveOut)); // bottom out
       // new Trigger(new JoyAxisBoolSupplier(joy1, 4, -0.5, false)).onTrue(new InstantCommand(intake :: TurnIn)); // intake in
      //  new Trigger(new JoyAxisBoolSupplier(joy1, 4, +0.5, true)).onTrue(new InstantCommand(intake :: TurnOut)); // intake out

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
        System.out.println("Starting teleop");
        startup.schedule();
        arm.enableBrake();
        drivetrain.resetEncoders();
    }

    public void disabledInit(){
        goodAuto = good.getBoolean(false);
        arm.disableBrake();
        drivetrain.init();
        drivetrain.stop();
        arm.armcontrolangle = false;
        arm.armcontrol = false;
    }

    public void stopEverything(){
        drivetrain.stop();
        // arm.stop();
    }

    public void startAuto(){
        if(autoCommandGroup != null) autoCommandGroup.cancel();
        drivetrain.resetEncoders();
        drivetrain.resetNAVx();
        goodAuto = good.getBoolean(false);
        //startup.schedule();
        System.out.println("Good auto? " + goodAuto + "!!!!!!!");
        drivetrain.stop();
        autoCommandGroup.schedule();
    }

    public void autoPeriod(){
       //drivetrain.UpdateOdometry();
       if(arm.armcontrol && arm.armcontrolangle) arm.updateController();
    }

    public void teleopPeriodic(){
         if(-joy1.getRawAxis(1) > 0.5){
             arm.turnUp();
         }
         else if(-joy1.getRawAxis(1) < -0.5){
             arm.turnDown();
        }

        if(joy1.getRawAxis(2) > 0.5){
            arm.topMoveIn();
         }
          if (joy1.getRawButton(5)){
             arm.bottomMoveIn();
         }

         if(joy1.getRawAxis(3) > 0.5){
            arm.topMoveOut();
         }
          if (joy1.getRawButton(6)){
             arm.bottomMoveOut();
         }

         if(-joy1.getRawAxis(5) < -0.5){
            intake.TurnIn();
         }
         else if(-joy1.getRawAxis(5) > 0.5){
            intake.TurnOut();
         }
         else {
            intake.StopMotor();
         }

         if(joy1.getPOV()>=0 && joy1.getPOV()<=180){
            intake.close();
         }
         else if(joy1.getPOV()<=360 && joy1.getPOV()>180 ){
            intake.open();
         }

        //drivetrain.UpdateOdometry();
        if(arm.armcontrol && arm.armcontrolangle) arm.updateController();
        double forward = -joy2.getRawAxis(JOY_Y_AXIS_ID); // negated because Y axis on controller is negated
        double turn = joy2.getRawAxis(JOY_Z_AXIS_ID);
        drivetrain.TankDrive(forward, turn);
    }
}
