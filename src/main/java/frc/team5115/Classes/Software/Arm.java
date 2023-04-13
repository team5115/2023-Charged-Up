package frc.team5115.Classes.Software;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Hardware.HardwareArm;
import frc.team5115.Classes.Hardware.HardwareIntake;
import edu.wpi.first.math.controller.PIDController;


public class Arm extends SubsystemBase{
    private HardwareArm intake;
    public HardwareIntake h;
    private double topLength = 0;
    private double bottomLength = 0;
    private double angle = -90;
    private double speed = 0.25;

    private double topKp = 0.113;
    private double bottomKp = 0.115;


    private PIDController turnController = new PIDController(0.08, 0.0, 0.0);
    public PIDController topWinchController = new PIDController(topKp, 0, 0);
    public PIDController bottomWinchController = new PIDController(bottomKp, 0, 0);

    public boolean armcontrol = true;
    public boolean armcontrolangle = true;

    public Arm(HardwareArm x, HardwareIntake y){
        intake = x;
        h =y;
        zeroArm();
        intake.setEncoders(topLength, -90);
    }

    public void setTopPID(double kP){
        topWinchController.setP(kP);
    }

    public void setBottomPID(double kP){
        bottomWinchController.setP(kP);
    }

    public void stepBottomPID(){
        setBottomPID(bottomWinchController.getP()*1.01);
    }

    public void slowBottomPID(){
        setBottomPID(bottomWinchController.getP()*0.99);
    }

    public void stepTopPID(){
        setTopPID(topWinchController.getP()*1.01);
    }

    public void slowTopPID(){
        setTopPID(topWinchController.getP()*0.99);
    }

    public void resetPID(){
        setTopPID(0.113);
        setBottomPID(0.115);
    }

    public void setTopWinchSpeed(){
        intake.setTopWinch(speed);
    }

    public void setLength(double length){
        topLength = length;
        bottomLength = length;
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

    public void turnSetAngle(double angle){
        this.angle = angle;
    }

    public void setArmUp(){
        //angle = 25.5;
        angle = 20;// for ff
        System.out.println("up");
    }

    public void setArmDown(){
        angle = -20;
        System.out.println("down");
    }

    public void turnUp() {
        angle += 9*0.02;
    }

    public void turnDown() {
        angle -= 9*0.02;
    }

    public void topMoveIn(){
        topLength += 10*0.02;
    }

    public void topMoveOut(){
        topLength -= 10*0.02;
    }

    public void bottomMoveIn(){
        bottomLength += 10*0.02;
    }

    public void bottomMoveOut(){
        bottomLength -= 10*0.02;
    }

    public void disableBrake(){
        intake.disableBrake();
    }

    public void enableBrake(){
        intake.enableBrake();
    }

    public void updateController(){
        if(bottomLength>5 || topLength>5){
            intake.FF = false;
        }
        else{
            intake.FF = true;
        }
        if(armcontrolangle) intake.setTurn(turnController.calculate(intake.getArmDeg(), angle));
        //System.out.println("Output Current" + intake.getTurnCurrent());
        //System.out.println("Current in Amps: " + intake.getTurnCurrent() + ", The Estimated Angle: "+  Math.round(getTurnDeg()) + ", and PID Value: "+ turnController.calculate(intake.getArmDeg(), angle));

        double bottomSpeed = bottomWinchController.calculate(intake.getBottomWinchLength(), bottomLength);
        double topSpeed = topWinchController.calculate(intake.getTopWinchLength(), topLength);
        //System.out.println("Top Length: " + intake.getTopWinchLength() + " Bottom Length: " + intake.getBottomWinchLength());
        //System.out.println("Top Current: " + intake.getTopCurrent() + "  Bottom Current: " + intake.getBottomCurrent() + " Turn Speed: " + turnController.calculate(intake.getArmDeg(), angle));
        if(armcontrol){
        intake.setTopWinch(topSpeed);
        intake.setBottomWinch(bottomSpeed);
        }
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
        intake.setEncoders(0, -100.0);
    }

    public void zeroLength(double angle){
        intake.setEncoders(0, angle);
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

    public double getAngle() {
        return intake.getArmDeg();
    }
}
