package frc.team5115.Robot;

import static frc.team5115.Constants.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team5115.Classes.Acessory.I2CHandler;
import frc.team5115.Classes.Hardware.*;
import frc.team5115.Classes.Software.*;
import frc.team5115.Commands.Auto.AutoCommandGroup;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.*;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    // private final Startup startup;
    private final ShuffleboardTab tab;
    private final GenericEntry center;
    private AutoCommandGroup autoCommandGroup;
    private boolean centerAuto = false;
    private I2CHandler i2cHandler;
    private final NAVx navx;
	private final Field2d field = new Field2d();

    private int printCounter = 0;

    public RobotContainer() {
        joy1 = new Joystick(0);
        joy2 = new Joystick(1);

        navx = new NAVx();
        i2cHandler = new I2CHandler();

        photonVision = new PhotonVision();
        intake = new HardwareIntake();
        hardwareArm = new HardwareArm(navx, i2cHandler);
        arm = new Arm(hardwareArm, intake);
        drivetrain = new Drivetrain(photonVision, arm, navx);
        // startup = new Startup(arm, hardwareArm, intake);
        
        tab = Shuffleboard.getTab("SmartDashboard");
        center = tab.add("Are we doing center balacing auto?", false).getEntry();

        timer = new Timer();
        timer.reset();
        configureButtonBindings();

		SmartDashboard.putData("Field", field);
    }

    public void configureButtonBindings() {
        new JoystickButton(joy2, XboxController.Button.kA.value).onTrue(new InstantCommand(drivetrain :: toggleSlowMode));
        
        new JoystickButton(joy1, XboxController.Button.kX.value).onTrue(new ShelfSubstation(arm)); // double substation pickup
        new JoystickButton(joy1, XboxController.Button.kY.value).onTrue(new HighNode(arm)); // high node
        new JoystickButton(joy1, XboxController.Button.kB.value).onTrue(new MiddleNode(arm)); // middle node
        new JoystickButton(joy1, XboxController.Button.kA.value).onTrue(new GroundPickup(arm)); // low node/ground pickup
        new JoystickButton(joy1, XboxController.Button.kStart.value).onTrue(new Stow(arm, hardwareArm, intake)); // stow fully
        new JoystickButton(joy1, XboxController.Button.kBack.value).onTrue(new StowCone(arm)); // stow with cone
    }

    public void startTeleop(){
        drivetrain.init();
        if(autoCommandGroup != null) autoCommandGroup.cancel();
        // arm.zeroArm();
        /* 
        System.out.println("Starting teleop");
        arm.enableBrake();
        startup.schedule();
        */
        drivetrain.resetEncoders();
    }

    public void disabledInit(){
        arm.disableBrake();
        drivetrain.stop();
        arm.armcontrolangle = false;
        arm.armcontrol = false;
        // i2cHandler.Disable();
    }

    public void stopEverything(){
        drivetrain.stop();
        // arm.stop();
    }

    public void startAuto(){
        if(autoCommandGroup != null) autoCommandGroup.cancel();
        drivetrain.resetEncoders();
        drivetrain.resetNAVx();
        drivetrain.stop();
        //startup.schedule();
        centerAuto = center.getBoolean(false);
        System.out.println("Good auto? " + centerAuto + "!!!!!!!");
        autoCommandGroup = new AutoCommandGroup(drivetrain, arm, hardwareArm, intake, centerAuto);
        autoCommandGroup.schedule();
    }

    public void autoPeriod(){
       //drivetrain.UpdateOdometry();
		arm.updateController();
		field.setRobotPose(drivetrain.getEstimatedPose());
    }

    public void teleopPeriodic(){

        if(-joy1.getRawAxis(XboxController.Axis.kLeftY.value) > 0.5){
            arm.turnUp();
        }
        else if(-joy1.getRawAxis(XboxController.Axis.kLeftY.value) < -0.5){
            arm.turnDown();
        }

        if(joy1.getRawButton(XboxController.Button.kLeftBumper.value)){
            arm.topMoveIn();
        }
        else if(joy1.getRawButton(XboxController.Button.kRightBumper.value)){
            arm.topMoveOut();
        }

        if(joy1.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.5){
            arm.bottomMoveIn();
        }
        else if(joy1.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.5){
            arm.bottomMoveOut();
        }

        final double rightY = -joy1.getRawAxis(XboxController.Axis.kRightY.value);
        if(rightY < -0.5){
            intake.TurnIn();
        }
        else if(rightY > 0.5){
            intake.TurnOut();
        }
        else {
            intake.StopMotor();
        }

        final double pov = joy1.getPOV();
        if(pov >= 0 && pov <= 180){
            intake.close();
        }
        else if(pov <= 360 && pov > 180 ){
            intake.open();
        }

        arm.updateController();
        // drivetrain.UpdateOdometry();
        // double forward = -joy2.getRawAxis(JOY_Y_AXIS_ID); // negated because Y axis on controller is negated
        // double turn = joy2.getRawAxis(JOY_Z_AXIS_ID);
        // drivetrain.TankDrive(forward, turn);
        
        // System.out.println(drivetrain.getEstimatedPose());

        printCounter++;
        if (printCounter % 20 == 0) {
            // right now it looks like the yawAddress is actually pitch
            // System.out.println(hardwareArm.getArmDeg());
            // System.out.println("Pitch: " + i2cHandler.getPitch());
            System.out.println("Yaw: " + i2cHandler.getYaw());
            // System.out.println("Roll: " + i2cHandler.getRoll());
        }

		field.setRobotPose(drivetrain.getEstimatedPose());
    }
}
