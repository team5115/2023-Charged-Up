package frc.team5115.Classes.Hardware;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The intake hardware subsystem. Provides methods to interact with the actual hardware of the intake.
 */
public class HardwareIntake extends SubsystemBase{
    private DoubleSolenoid intake;
    private PneumaticsControlModule pcm;
    private TalonSRX intakeL = new TalonSRX(9);
    private TalonSRX intakeR = new TalonSRX(8);
    //PCM IS 10 this season YOU HAVE TO LABEL THE MODULE/CAN ID in everything you instantiate
    private final DigitalOutput coneLight;
    private final DigitalOutput cubeLight;
    public boolean open = true;

	/**
	 * `HardwareIntake` constructor.
	 */
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

	/**
	 * Spins the intake wheels in.
	 */
    public void TurnIn(){
        intakeL.set(ControlMode.PercentOutput, -0.7);
        intakeR.set(ControlMode.PercentOutput, -0.7);
    }

	/**
	 * Spins the intake wheels out.
	 */
    public void TurnOut(){
        intakeL.set(ControlMode.PercentOutput, +0.7);
        intakeR.set(ControlMode.PercentOutput, +0.7);
    }

	/**
	 * Stops the intake wheels.
	 */
    public void StopMotor(){
        intakeL.set(ControlMode.PercentOutput, -0.09);
        intakeR.set(ControlMode.PercentOutput,-0.09);
    }

	/**
	 * Puts the intake into cube mode.
	 */
    public void open(){
        intake.set(Value.kReverse);
        setLights(true);
        open = true;
    }

	/**
	 * Puts the intake into cone mode.
	 */
    public void close(){
        intake.set(Value.kForward);
        setLights(false);
        open = false;
    }

	/**
	 * Sets the indicator light on the top of the robot to display whether it wants a cone or a cube.
	 * @param wantsCones - Whether the robot wants a cone or a cube. True for cone, false for cube
	 */
    private void setLights(boolean wantsCones) {
        coneLight.set(wantsCones);
        cubeLight.set(!wantsCones);
        //System.out.println(wantsCones);
    }
    
}