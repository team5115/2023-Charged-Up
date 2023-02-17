package frc.team5115.Classes.Software;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Hardware.HardwareArm;
import edu.wpi.first.math.controller.PIDController;

public class Arm extends SubsystemBase{
    private HardwareArm intake;
    private double topLength = 23;
    private double bottomLength = 23;
    private double angle = 40;
    private double speed = 0.25;
    private PIDController turnController = new PIDController(0.05, 0.0001, 0.02);
    private PIDController topWinchController = new PIDController(0.075, 0, 0);
    private PIDController bottomWinchController = new PIDController(0.045, 0, 0);

    public Arm(){
        intake = new HardwareArm();
        zeroArm();
        intake.setEncoders(topLength);
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
        bottomLength = 10;      
        topLength = 10;  
        System.out.println("in: " + bottomLength);
    }

    public void Out(){
        bottomLength = 20;      
        topLength = 20;  
        System.out.println("out" + bottomLength);
    }

    public void turnController(){
        //intake.setTurn(0.10);
        intake.setTurn(turnController.calculate(intake.getArmDeg(), angle));
        //System.out.println("Output Current" + intake.getTurnCurrent());
        //System.out.println("Current in Amps: " + intake.getTurnCurrent() + ", The Estimated Angle: "+  Math.round(getTurnDeg()) + ", and PID Value: "+ turnController.calculate(intake.getArmDeg(), angle));
        
        double bottomSpeed = bottomWinchController.calculate(intake.getBottomWinchLength(), bottomLength);
        double topSpeed = topWinchController.calculate(intake.getTopWinchLength(), topLength);
        //System.out.println("Top Length: " + intake.getTopWinchLength() + " Bottom Length: " + intake.getBottomWinchLength());
        //System.out.println("Turn Current: " + intake.getTurnCurrent() + " Top Current: " + intake.getTopCurrent() + "  Bottom Current: " + intake.getBottomCurrent() + " Turn Speed: " + turnController.calculate(intake.getArmDeg(), angle));
        
        if(intake.getTopWinchLength() - intake.getBottomWinchLength()>1){
            System.out.println("Top too far ahead & ");
                if(topSpeed< 0 && bottomSpeed<0){
                    System.out.print("Bottom Stopped");
                    intake.setBottomWinch(bottomSpeed);
                    intake.setTopWinch(0);
                }
                else if(topSpeed<0 && bottomSpeed>0){
                    System.out.print("Both Stopped");
                    intake.setTopWinch(0);
                    intake.setBottomWinch(bottomSpeed);
                }
                else if(topSpeed>0  && bottomSpeed>0){
                    System.out.print("Top Stopped");
                    intake.setTopWinch(0);
                    intake.setBottomWinch(bottomSpeed);
                }
                else{
                    System.out.print("Neither Stopped");
                    intake.setTopWinch(topSpeed);
                    intake.setBottomWinch(bottomSpeed);
                }
            }
            else if(intake.getBottomWinchLength() - intake.getTopWinchLength() >1){
                System.out.println("Bottom too far ahead");
                if(topSpeed< 0 && bottomSpeed<0){
                    System.out.print("Bottom Stopped");
                    intake.setTopWinch(0);
                    intake.setBottomWinch(bottomSpeed);
                }
                else if(topSpeed>0 && bottomSpeed<0){
                    System.out.print("Neither Stopped");
                    intake.setTopWinch(topSpeed);
                    intake.setBottomWinch(bottomSpeed);
                }
                else if(topSpeed>0  && bottomSpeed>0){
                    System.out.print("Top Stopped");
                    intake.setTopWinch(topSpeed);
                    intake.setBottomWinch(0);
                }
                else{
                    System.out.print("Both Stopped");
                    intake.setTopWinch(0);
                    intake.setBottomWinch(0);
                }
            }
        else{
            System.out.println("Everything fine");
        intake.setTopWinch(topSpeed);
        intake.setBottomWinch(bottomSpeed);
        }
    }

    public void turnSetAngle(double angle){
        this.angle = angle;
    }

    public void setArmIn(){
        angle = 30;
        System.out.println("in");
    }

    public void setArmOut(){
        angle = 15;
        System.out.println("out");
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
