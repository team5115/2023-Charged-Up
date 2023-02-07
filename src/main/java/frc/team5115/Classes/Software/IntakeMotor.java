package frc.team5115.Classes.Software;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Hardware.HardwareIntakeMotor;
import edu.wpi.first.math.controller.PIDController;

public class IntakeMotor extends SubsystemBase{
    private HardwareIntakeMotor intake;
    private PIDController turnController = new PIDController(0, 0, 0);
    private PIDController topWinchController = new PIDController(0, 0, 0);
    private PIDController bottomWinchController = new PIDController(0, 0, 0);

    public IntakeMotor(){
        intake = new HardwareIntakeMotor();
    }

    public void topWinchController(double length){
        topWinchController.calculate(intake.getTopWinchLength(), length);
    }

    public void bottomWinchController(double length){
        bottomWinchController.calculate(intake.getBottomWinchLength(), length);
    }

    public void turnController(double length){
        turnController.calculate(intake.getArmDeg(), length);
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

    public boolean getFault(CANSparkMax.FaultID f){
        return intake.getFault(f);
    }

    public void stop(){
        intake.stop();
    }
}
