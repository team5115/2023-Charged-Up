package frc.team5115.Classes.Hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HardwareIntakeMotor extends SubsystemBase{
    private CANSparkMax intakeTop;
    private CANSparkMax intakeBottom;
    private CANSparkMax intakeTurn;
    private RelativeEncoder TurningEncoder;
    private RelativeEncoder TopWinchEncoder;
    private RelativeEncoder BottomWinchEncoder;
    //private double encoderConstant = 1/49;
    private double startingTurnValue = -90;
    private double rotatingGearRatio = ((1/42)*(10/48));
    private double winchGearRatio = 1/7;
    private double WinchDiameter = 0.75; //Only example of in, easier to determine length of the rod

    public HardwareIntakeMotor(){
        intakeTop = new CANSparkMax(5, MotorType.kBrushless);   
        intakeBottom = new CANSparkMax(6, MotorType.kBrushless);    
        intakeTurn = new CANSparkMax(7, MotorType.kBrushless);    
 
        TurningEncoder = intakeTurn.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        TopWinchEncoder = intakeTop.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        BottomWinchEncoder = intakeTop.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
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

    public double getTurnCurrent(){
        return intakeTurn.getOutputCurrent();
    }
    
    public double getTurnEncoder(){
        return (TurningEncoder.getPosition());
    }

    public double getTopEncoder(){
        return (TopWinchEncoder.getPosition());
    }

    public double getBottomEncoder(){
        return (BottomWinchEncoder.getPosition());
    }

    public double getVelocity(){
        return TurningEncoder.getVelocity();
    }

    public boolean getFault(CANSparkMax.FaultID f){
        return intakeTurn.getFault(f);
    }

    public void resetEncoders(){
        TurningEncoder.setPosition(startingTurnValue);
        BottomWinchEncoder.setPosition(0);
        TopWinchEncoder.setPosition(0);
    }

    public void resetEncoders(Double angle){
        TurningEncoder.setPosition(angle);
        BottomWinchEncoder.setPosition(0);
        TopWinchEncoder.setPosition(0);
    }

    public double getTopWinchLength() {
        return getTopEncoder()*winchGearRatio*(WinchDiameter*2*Math.PI);
    }

    public double getBottomWinchLength() {
        return getBottomEncoder()*winchGearRatio*(WinchDiameter*2*Math.PI);
    }

    public double getArmDeg(){
       return (getTurnEncoder())*(360/(rotatingGearRatio))-startingTurnValue;
    } 

    public double getArmRad(){
        return (getArmDeg()/180)*Math.PI;
    }

    public double getBCenterOfMass(double length){
        return (4917*length+11.1384);
    }

    public double getSCenterOfMass(double length){
    return (.34831*length+14.3416);
    }

    public double getSmallTorque(double length){
        return 2.584*getSCenterOfMass(length)*Math.sin(getArmRad());
    }

    public double getBigTorque(double length){
        return 4.4104*getBCenterOfMass(length)*Math.sin(getArmRad());
    }

    public double getMassTorque(double mass, double length){
        return mass * length * Math.sin(getArmRad());
    }

    public double getTotalTorque(double mass, double length){
        return (getSmallTorque(length) + getBigTorque(length) + getMassTorque(mass, length)/8.85);
    }

}
