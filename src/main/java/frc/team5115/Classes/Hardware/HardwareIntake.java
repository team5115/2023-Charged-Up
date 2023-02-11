package frc.team5115.Classes.Hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team5115.Constants.*;

public class HardwareIntake extends SubsystemBase{
    private TalonSRX intake;
    private double intakeSpeed = .2;

    public HardwareIntake(){
        intake = new TalonSRX(8);    
    }

    public void forwardIntake(){
        intake.set(ControlMode.PercentOutput, intakeSpeed);
    }

    public void reverseIntake(){
        intake.set(ControlMode.PercentOutput, -intakeSpeed);
    }

    public void stop(){
        intake.set(ControlMode.PercentOutput, 0);
    }
    
}