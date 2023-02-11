package frc.team5115.Classes.Hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HardwareArm extends SubsystemBase{
    private CANSparkMax intakeTop;
    private CANSparkMax intakeBottom;
    private CANSparkMax intakeTurn;
    private RelativeEncoder TurningEncoder;
    private RelativeEncoder TopWinchEncoder;
    private RelativeEncoder BottomWinchEncoder;
    //private double encoderConstant = 1/49;
    private double startingTurnValue = 0;
    private double rotatingGearRatio = ((1/49)*(10/48));
    private double winchGearRatio = 1/7;
    private double WinchDiameter = 0.75; //Only example of in, easier to determine length of the rod

    public HardwareArm(){
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
        double turnSpeed = Math.min(Math.max(speed, -0.15), .15);
        intakeTurn.set(turnSpeed);
    }

    public void stop(){
        //setTopWinch(0);
        //setBottomWinch(0);
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

    public double getTurnVelocity(){
        return (TurningEncoder.getVelocity());
    }

    public double getTopVelocity(){
        return (TopWinchEncoder.getVelocity());
    }

    public double getBottomVelocity(){
        return (BottomWinchEncoder.getVelocity());
    }


    public String getEncoders(){
        return ("Bottom Winch Val:" + BottomWinchEncoder.getPosition() + " Top Winch Val:" + TopWinchEncoder.getPosition() + " Turn Motor Val:" + TurningEncoder.getPosition());
    }

    public String getVelocities(){
        return ("Bottom Winch Velocity:" + BottomWinchEncoder.getVelocity() + " Top Winch Val:" + TopWinchEncoder.getVelocity() + " Turn Motor Val:" + TurningEncoder.getVelocity());
    }

    public boolean getFault(CANSparkMax.FaultID f){
        return intakeTurn.getFault(f);
    }

    public void zeroEncoders(){
        TurningEncoder.setPosition(startingTurnValue);
        BottomWinchEncoder.setPosition(0);
        TopWinchEncoder.setPosition(0);
    }

    /** 
     * Returns the length of the Top Winch
     * @return the length of the Top Winch, converted from rots to in
     */
    public double getTopWinchLength() {
        return getTopEncoder()*winchGearRatio*(WinchDiameter*2*Math.PI);
    }

    /** 
     * Returns the length of the Bottom Winch
     * @return the length of the Bottm Winch, converted from rots to in
     */
    public double getBottomWinchLength() {
        return getBottomEncoder()*winchGearRatio*(WinchDiameter*2*Math.PI);
    }

    /** 
     * Returns the angle of the turning arm
     * @return the angle the arm turned
     */

    public double getArmDeg(){
       return (getTurnEncoder())*(360/(rotatingGearRatio));
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
