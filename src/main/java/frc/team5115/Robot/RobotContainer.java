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
import frc.team5115.Commands.Intake.CombinedIntakeCommands.*;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;

public class RobotContainer {
    private final Timer timer;
    private final Joystick joy1;
    private final Joystick joy2;
    private final PhotonVision photonVision;
    private final Drivetrain drivetrain;
    private final HardwareIntake intake;
    private final Arm arm;
    private final HardwareArm hardwareArm;
    private AutoCommandGroup autoCommandGroup;
    private final DockCommandGroup dockSequence;
    private Startup_Intake startup;
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
        startup = new Startup_Intake(arm, hardwareArm, intake);        
        dockSequence = new DockCommandGroup(drivetrain, false);
        timer = new Timer();
        timer.reset();
        configureButtonBindings();
    }

    public void configureButtonBindings() {
        new JoystickButton(joy2, 1).onTrue(new InstantCommand(drivetrain :: toggleSlowMode));
        
        new JoystickButton(joy1, 3).onTrue(new ShelfSubstation(arm)); // double substation pickup
        new JoystickButton(joy1, 4).onTrue(new HighNode(arm)); // high node
        new JoystickButton(joy1, 2).onTrue(new MiddleNode(arm)); // middle node
        new JoystickButton(joy1, 1).onTrue(new GroundPickup(arm)); // low node/ground pickup
        new JoystickButton(joy1, 8).onTrue(new Stow(arm, hardwareArm, intake)); // stow fully
        new JoystickButton(joy1, 7).onTrue(new StowCone(arm, hardwareArm, intake)); // stow with cone
        
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
        autoCommandGroup = new AutoCommandGroup(drivetrain, arm, hardwareArm, intake, goodAuto);
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
