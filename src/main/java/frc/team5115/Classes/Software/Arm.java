package frc.team5115.Classes.Software;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Hardware.HardwareArm;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Arm extends SubsystemBase{
    private HardwareArm intake;
    private double topLength = 0;
    private double bottomLength = 0;
    private double angle = -30;
    private double speed = 0.25;
    private ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
    private GenericEntry topKp = tab.add("topKp", 0.05).getEntry();
    private GenericEntry bottomKp = tab.add("bottomKp", 0.04).getEntry();
    private GenericEntry topAngle = tab.add("topAngle", 0).getEntry();
    private PIDController turnController = new PIDController(0.06, 0.0, 0.0);
    public PIDController topWinchController = new PIDController(topKp.getDouble(0.05), 0, 0);
    public PIDController bottomWinchController = new PIDController(bottomKp.getDouble(0.05), 0, 0);
    public boolean armcontrol = false;

    public Arm(HardwareArm x){
        intake = x;
        zeroArm();
        intake.setEncoders(topLength, -90);
    }

    public void setTopWinchSpeed(){
        intake.setTopWinch(speed);
    }

    public void setNegTopWinchSpeed(){
        intake.setTopWinch(-speed);
    }

    public void setBottomWinchSpeed(){
        intake.setBottomWinch(speed);
    }

    public void setNegBottomWinchSpeed(){
        intake.setBottomWinch(-speed);
    }

    public void setTurnSpeed(){
        intake.setTurn(speed);
    }

    public void topWinchController(){
        topWinchController.calculate(intake.getTopWinchLength(), topLength);
    }

    public void topWinchSetLength(double length){
        this.topLength = length;
    }

    public void bottomWinchController(){
        bottomWinchController.calculate(intake.getBottomWinchLength(), bottomLength);
    }

    public void bottomWinchSetLength(double length){
        this.bottomLength = length;
    }

    public void In(){
        bottomLength = 1;      
        topLength = 0.5;  
        intake.FF = true;
        System.out.println("in: " + bottomLength);
    }

    public void Out(){
        bottomLength = 23;
        topLength = 21;  
        intake.FF = false;
        System.out.println("out" + bottomLength);
    }

    public void Reset(){
        bottomLength = 26;
        topLength = 25;
        intake.FF = false;
    }

    public void turnSetAngle(double angle){
        this.angle = angle;
    }

    public void setArmUp(){
        //angle = 25.5;
        angle = 15;// for ff
        System.out.println("up");
    }

    public void setArmDown(){
        angle = -20;
        System.out.println("down");
    }

    public void setArmStart(){
        angle = -90;
    }

    public void turnUp() {
        angle += 3*0.02;
    }

    public void turnDown() {
        angle -= 3*0.02;
    }

    public void updateController(){
        if(bottomLength>5 || topLength>5){
            intake.FF = false;
        }
        else{
            intake.FF = true;
        }
        //intake.setTurn(0.10);
        intake.setTurn(turnController.calculate(intake.getArmDeg(), angle));
        //System.out.println("Output Current" + intake.getTurnCurrent());
        //System.out.println("Current in Amps: " + intake.getTurnCurrent() + ", The Estimated Angle: "+  Math.round(getTurnDeg()) + ", and PID Value: "+ turnController.calculate(intake.getArmDeg(), angle));

        double bottomSpeed = bottomWinchController.calculate(intake.getBottomWinchLength(), bottomLength);
        double topSpeed = topWinchController.calculate(intake.getTopWinchLength(), topLength);
        //System.out.println("Top Length: " + intake.getTopWinchLength() + " Bottom Length: " + intake.getBottomWinchLength());
        //System.out.println("Top Current: " + intake.getTopCurrent() + "  Bottom Current: " + intake.getBottomCurrent() + " Turn Speed: " + turnController.calculate(intake.getArmDeg(), angle));

        intake.setTopWinch(topSpeed);
        intake.setBottomWinch(bottomSpeed);
    }

    public double getTurnDeg(){
        return intake.getArmDeg();
    }

    public double getTopWinchLength(){
        return intake.getTopWinchLength();
    }

    public double getBottomWinchLength(){
        return intake.getBottomWinchLength();
    }
    
    public double getArmSpeed(){
        return intake.getTurnVelocity();
    }

    public void zeroArm(){
        intake.setEncoders(0, -90);
    }

    public boolean getFault(CANSparkMax.FaultID f){
        return intake.getFault(f);
    }

    public void stop(){
        intake.stop();
    }

    public void moveTop(){
        intake.setTopWinch(-0.2);
    }

    public void moveBottom(){
        intake.setBottomWinch(-0.2);
    }
}
