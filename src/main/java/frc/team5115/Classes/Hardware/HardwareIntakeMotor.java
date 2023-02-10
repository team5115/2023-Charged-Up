package frc.team5115.Classes.Hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HardwareIntakeMotor extends SubsystemBase{
    private CANSparkMax intakeTop;
    private CANSparkMax intakeBottom;
    private CANSparkMax intakeTurn;

    public HardwareIntakeMotor(){
        intakeTop = new CANSparkMax(5, MotorType.kBrushless);   
        intakeBottom = new CANSparkMax(6, MotorType.kBrushless);    
        intakeTurn = new CANSparkMax(7, MotorType.kBrushless);    
 
    }

    public void setTopWinch(double speed){
        intakeTop.set(speed);
    }

    public void setBottomWinch(double speed){
        intakeBottom.set(speed);
    }

    public void setTurn(double speed){
        intakeTurn.set(speed);
    }

    public void stop(){
        setTopWinch(0);
        setBottomWinch(0);
        setTurn(0);
    }

}
