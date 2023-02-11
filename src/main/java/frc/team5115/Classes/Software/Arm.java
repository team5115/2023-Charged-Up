package frc.team5115.Classes.Software;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Hardware.HardwareArm;
import edu.wpi.first.math.controller.PIDController;

public class Arm extends SubsystemBase{
    private HardwareArm intake;
    private double topLength = 0;
    private double bottomLength = 0;
    private double angle = 0;
    private double speed = 0.15;
    private PIDController turnController = new PIDController(0.01, 0, 0);
    private PIDController topWinchController = new PIDController(0, 0, 0);
    private PIDController bottomWinchController = new PIDController(0, 0, 0);

    public Arm(){
        intake = new HardwareArm();
    }

    public void setTopWinchSpeed(){
        intake.setTopWinch(speed);
    }

    public void setBottomWinchSpeed(){
        intake.setBottomWinch(speed);
    }

    public void setTurnSpeed(){
        double turnSpeed = Math.min(Math.max(speed, -0.15), .15);
        intake.setTurn(turnSpeed);
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

    public void turnController(){
        intake.setTurn(turnController.calculate(intake.getArmDeg(), angle));
    }

    public void turnSetAngle(double angle){
        this.angle = angle;
    }

    public void updateController(){
        turnController();
        //topWinchController();
        //bottomWinchController();
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
        intake.zeroEncoders();
    }

    public boolean getFault(CANSparkMax.FaultID f){
        return intake.getFault(f);
    }

    public void stop(){
        intake.stop();
    }
}
