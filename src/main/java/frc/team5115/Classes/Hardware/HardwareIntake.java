package frc.team5115.Classes.Hardware;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class HardwareIntake extends SubsystemBase{
    private DoubleSolenoid intake;
    private PneumaticsControlModule pcm;
    //PCM IS 10 this season YOU HAVE TO LABEL THE MODULE/CAN ID in everything you instantiate

    public HardwareIntake(){
        pcm = new PneumaticsControlModule(10);
        intake = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 0, 1);
    }

    public void open(){
        intake.set(Value.kForward);
    }

    public void close(){
        intake.set(Value.kReverse);
    }
    
}