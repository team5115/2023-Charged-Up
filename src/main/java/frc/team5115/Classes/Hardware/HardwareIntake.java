package frc.team5115.Classes.Hardware;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class HardwareIntake extends SubsystemBase{
    private DoubleSolenoid intake;
    private PneumaticsControlModule pcm;
    private TalonSRX intakeL = new TalonSRX(9);
    private TalonSRX intakeR = new TalonSRX(8);
    //PCM IS 10 this season YOU HAVE TO LABEL THE MODULE/CAN ID in everything you instantiate

    public HardwareIntake(){
        pcm = new PneumaticsControlModule(10);
        intake = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 0, 1);
    }

    public void open(){
        intake.set(Value.kReverse);
    }

    public void TurnIn(){
        intakeL.set(ControlMode.PercentOutput, +0.7);
        intakeR.set(ControlMode.PercentOutput, +0.7);
    }

    public void TurnOut(){
        intakeL.set(ControlMode.PercentOutput, -0.7);
        intakeR.set(ControlMode.PercentOutput, -0.7);
    }

    public void StopMotor(){
        intakeL.set(ControlMode.PercentOutput, 0);
        intakeR.set(ControlMode.PercentOutput,0);
    }

    public void close(){
        intake.set(Value.kForward);
    }
    
}