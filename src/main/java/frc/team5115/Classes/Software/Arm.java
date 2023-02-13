package frc.team5115.Classes.Software;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Hardware.HardwareArm;
import edu.wpi.first.math.controller.PIDController;

public class Arm extends SubsystemBase{
    private HardwareArm intake;
    private double topLength = 0;
    private double bottomLength = 0;
    private double angle = 40;
    private double speed = 0.25;
    private PIDController turnController = new PIDController(0.029, 0.0005, 0.01);
    private PIDController topWinchController = new PIDController(0, 0, 0);
    private PIDController bottomWinchController = new PIDController(0, 0, 0);

    public Arm(){
        intake = new HardwareArm();
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

    public void turnController(){
        intake.setTurn(turnController.calculate(intake.getArmDeg(), angle));
        System.out.println("Output Current" + intake.getTurnCurrent());
        System.out.println("Encoder Value: " + Math.round(intake.getTurnEncoder()) + ", The Estimated Angle: "+  Math.round(getTurnDeg()) + ", and PID Value: "+ turnController.calculate(intake.getArmDeg(), angle));
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
