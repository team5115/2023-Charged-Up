package frc.team5115.Classes.Software;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Hardware.HardwareIntakeMotor;

public class IntakeMotor extends SubsystemBase{
    private HardwareIntakeMotor intake;
  
    public IntakeMotor(){
        intake = new HardwareIntakeMotor();
    }

    public void stop(){
        intake.stop();
    }
}
