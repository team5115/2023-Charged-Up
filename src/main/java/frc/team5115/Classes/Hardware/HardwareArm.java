package frc.team5115.Classes.Hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Acessory.I2CHandler;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.util.Units;

public class HardwareArm extends SubsystemBase{
    private CANSparkMax intakeTop;
    private CANSparkMax intakeBottom;
    private CANSparkMax intakeTurn;
    private RelativeEncoder TurningEncoder;
    private RelativeEncoder TopWinchEncoder;
    private RelativeEncoder BottomWinchEncoder;
    private final double Ks = 0.13;
    private final double Kv = 4.5;
    private final double Ka = 0.1113;
    private final double Kg = 0.39;
    public boolean FF = true;
    private final ArmFeedforward arm = new ArmFeedforward(Ks, Kg, Kv, Ka); // Rad Calibrated
    //private double encoderConstant = 1/49;
    private double startingTurnValue = Units.degreesToRadians(-96); //Rads
    private double WinchDiameter = Units.metersToInches(0.12); 
    private final NAVx navx;
    private final I2CHandler i2cHandler;

    public HardwareArm(NAVx nav, I2CHandler i2c){
        intakeTop = new CANSparkMax(5, MotorType.kBrushless);   
        intakeBottom = new CANSparkMax(6, MotorType.kBrushless);    
        intakeTurn = new CANSparkMax(7, MotorType.kBrushless);  

        intakeBottom.setInverted(true);
        intakeTop.setInverted(true);

        intakeTop.setIdleMode(IdleMode.kBrake);
        intakeBottom.setIdleMode(IdleMode.kBrake);
        intakeTurn.setIdleMode(IdleMode.kBrake);

        navx = nav;
        i2cHandler = i2c;
        
        //intakeTop.setSmartCurrentLimit(40, 40);
        //intakeBottom.setSmartCurrentLimit(40, 40);
        intakeTurn.setSmartCurrentLimit(80, 80);

        //intakeBottom.follow(intakeTop);
 
        TurningEncoder = intakeTurn.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        TopWinchEncoder = intakeTop.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        BottomWinchEncoder = intakeBottom.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

        TurningEncoder.setPositionConversionFactor(1);
        TopWinchEncoder.setPositionConversionFactor(1);
        BottomWinchEncoder.setPositionConversionFactor(1);
        TurningEncoder.setVelocityConversionFactor(1);
        TopWinchEncoder.setVelocityConversionFactor(1);
        BottomWinchEncoder.setVelocityConversionFactor(1);

    }

    public void setTopWinch(double speed){
        // NaN Check
        if(speed != speed) {
            speed = 0;
        }

        intakeTop.set(speed);
    }

    public void setBottomWinch(double speed){
        // NaN Check
        if(speed != speed) {
            speed = 0;
        }

        intakeBottom.set(speed);
    }

    public void setTurn(double speed){
        //TURN ONLY IF THE ARMS ARE WITHDREW

        if(speed != speed) {
            speed = 0;
        }
        if(FF){
            intakeTurn.setVoltage(Math.max(arm.calculate((getArmRad()), 1.5*speed), -10));
        }
        else{
            if(speed>.37){
                speed = 0.37;
            }
            else if(speed<-0.2){
                speed = -0.2;
            }
            intakeTurn.set(speed);
        }
    }

    public void stop(){
        setTopWinch(0);
        setBottomWinch(0);
        setTurn(0);
    }
    
    public double getTurnCurrent(){
        return intakeTurn.getOutputCurrent();
    }

    public double getTopCurrent(){
        return intakeTop.getOutputCurrent();
    }
    
    public double getBottomCurrent(){
        return intakeBottom.getOutputCurrent();
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
        return -(TopWinchEncoder.getVelocity());
    }

    public double getBottomVelocity(){
        return -(BottomWinchEncoder.getVelocity());
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
        BottomWinchEncoder.setPosition(0);
        TopWinchEncoder.setPosition(0);
        TurningEncoder.setPosition(0);
    }

public void setEncoders(double Length, double angle){
    BottomWinchEncoder.setPosition((7*Length)/((WinchDiameter)));
    TopWinchEncoder.setPosition((7*Length)/((WinchDiameter)));
    TurningEncoder.setPosition(angle/(360.0 / (48.0 * 49.0 / 10.0))); // Set the angle 
    // TurningEncoder.setPosition(Units.radiansToDegrees(startingTurnValue)/(360.0 / (48.0 * 49.0 / 10.0)));
}

    /** 
     * Returns the length of the Top Winch
     * @return the length of the Top Winch, converted from rots to in
     */
    public double getTopWinchLength() {
        //System.out.println((getTopEncoder()/7)*(WinchDiameter*3.14159));
        //System.out.println((getBottomEncoder()/7)*(WinchDiameter));
        return (getTopEncoder()/7)*(WinchDiameter);
    
    }

    /** 
     * Returns the length of the Bottom Winch
     * @return the length of the Bottm Winch, converted from rots to in
     */
    public double getBottomWinchLength() {
        //System.out.println((getBottomEncoder()/7)*(WinchDiameter*3.14159));
        //System.out.println((getBottomEncoder()/7)*(WinchDiameter));
        return (getBottomEncoder()/7)*(WinchDiameter);
    }

    /** 
     * Returns the angle of the turning arm
     * @return the angle the arm turned
     */

    public double getArmDeg(){
        // return getTurnEncoder() * (360.0 / (48.0 * 49.0 / 10.0));
        return getAbsArmDeg();
    }

    public double getAbsArmDeg() {
        final double navxPitch = navx.getPitchDeg();
        final double bnoPitch = i2cHandler.getPitch();
        return bnoPitch - navxPitch;
    }

    public double getArmRad(){
        //return Math.toRadians(getArmDeg());
        return Math.toRadians(getArmDeg() + startingTurnValue);
    }

    public void disableBrake(){
        intakeBottom.setIdleMode(IdleMode.kCoast);
        intakeTop.setIdleMode(IdleMode.kCoast);
        //intakeTurn.setIdleMode(IdleMode.kCoast);
    }

    public void enableBrake(){
        intakeBottom.setIdleMode(IdleMode.kBrake);
        intakeTop.setIdleMode(IdleMode.kBrake);
        intakeTurn.setIdleMode(IdleMode.kBrake);
    }

}
