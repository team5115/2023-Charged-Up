package frc.team5115.Classes.Software;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Hardware.HardwareIntakeMotor;
import edu.wpi.first.math.controller.PIDController;

public class IntakeMotor extends SubsystemBase{
    private HardwareIntakeMotor intake;
    private double topLength = 0;
    private double bottomLength = 0;
    private double angle = -90;
    private PIDController turnController = new PIDController(0, 0, 0);
    private PIDController topWinchController = new PIDController(0, 0, 0);
    private PIDController bottomWinchController = new PIDController(0, 0, 0);

    public IntakeMotor(){
        intake = new HardwareIntakeMotor();
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
        turnController.calculate(intake.getArmDeg(), angle);
    }

    public void turnSetAngle(double angle){
        this.angle = angle;
    }

    public void updateController(){
        turnController();
        topWinchController();
        bottomWinchController();
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
    
    public double getSpeed(){
        return intake.getVelocity();
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
