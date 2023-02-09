package frc.team5115.Classes.Hardware;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Pneumatic extends SubsystemBase{
    private DoubleSolenoid intake;
    private PneumaticsControlModule pcm;

    public Pneumatic(){
        pcm = new PneumaticsControlModule(10);
        intake = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 1, 0);
    }

    public void open(){
        
        System.out.println(intake.isFwdSolenoidDisabled());
         intake.set(Value.kForward);
    }

    public void close(){
        System.out.println(intake.isFwdSolenoidDisabled());
         intake.set(Value.kReverse);

    }
    
}