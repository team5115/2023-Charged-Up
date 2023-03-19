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
    private final DigitalOutput coneLight;
    private final DigitalOutput cubeLight;

    public HardwareIntake(){
        intakeL.configPeakCurrentLimit(35);
        intakeL.enableCurrentLimit(true);
        intakeR.configPeakCurrentLimit(35);
        intakeR.enableCurrentLimit(true);
        pcm = new PneumaticsControlModule(10);
        intake = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 0, 1);
        coneLight = new DigitalOutput(0);
        coneLight.set(true);
        cubeLight = new DigitalOutput(1);
    }

    public void TurnIn(){
        intakeL.set(ControlMode.PercentOutput, -0.7);
        intakeR.set(ControlMode.PercentOutput, -0.7);
    }

    public void TurnOut(){
        intakeL.set(ControlMode.PercentOutput, +0.7);
        intakeR.set(ControlMode.PercentOutput, +0.7);
    }

    public void StopMotor(){
        intakeL.set(ControlMode.PercentOutput, -0.09);
        intakeR.set(ControlMode.PercentOutput,-0.09);
    }

    public void open(){
        intake.set(Value.kReverse);
        setLights(true);
    }

    public void close(){
        intake.set(Value.kForward);
        setLights(false);
    }

    private void setLights(boolean wantsCones) {
        coneLight.set(wantsCones);
        cubeLight.set(!wantsCones);
        System.out.println(wantsCones);
    }
    
}