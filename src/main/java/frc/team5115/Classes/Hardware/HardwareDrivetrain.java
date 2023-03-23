package frc.team5115.Classes.Hardware;

import static frc.team5115.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.MathUtil;

public class HardwareDrivetrain{
    // Competition feedforward and feedback (pid) values
    // 6 inch diameter on COMP ROBOT WITH ARM and dumbells in back
    // private static final double leftKs = 0.17463;
    // private static final double leftKv = 2.8104;
    // private static final double leftKa = 0.82143;
    
    // private static final double rightKs = 0.17463;
    // private static final double rightKv = 2.8104;
    // private static final double rightKa = 0.82143;

    // private static final double leftKp = 1.6455;
    // private static final double rightKp = 1.6220;
    // private static final double Ki = 0.0;
    // private static final double Kd = 0.0;
    // // END of comp robot values

    // Testbed feedforward and feedback (pid) values - 6 inch diameter on testbed
    private static final double leftKs = 0.090949;
    private static final double leftKv = 2.783;
    private static final double leftKa = 0.16477;
    
    private static final double rightKs = 0.099706;
    private static final double rightKv = 2.8314;
    private static final double rightKa = 0.14565;
    
    private static final double leftKp = 0.0; // 3.7203 according to sysid
    private static final double rightKp = 0.0; // 3.7203 according to sysid
    private static final double Ki = 0.0;
    private static final double Kd = 0.0;
    // END of testbed values

    private final SimpleMotorFeedforward leftFeedForward = new SimpleMotorFeedforward(leftKs, leftKv, leftKa);
    private final SimpleMotorFeedforward rightFeedForward = new SimpleMotorFeedforward(rightKs, rightKv, rightKa);
    private final PIDController leftPID = new PIDController(leftKp, Ki, Kd);
    private final PIDController rightPID = new PIDController(rightKp, Ki, Kd);

    private final CANSparkMax frontLeft = new CANSparkMax(FRONT_LEFT_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax frontRight = new CANSparkMax(FRONT_RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax backLeft = new CANSparkMax(BACK_LEFT_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax backRight = new CANSparkMax(BACK_RIGHT_MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder leftEncoder = frontLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    private final RelativeEncoder rightEncoder = frontRight.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    public HardwareDrivetrain(){
        resetEncoders();
        frontRight.setInverted(true);
    }

    /**
     * @param motorID of the motor to get an encoder of.
     * @return a reference to an encoder matching the id
     */
    public RelativeEncoder getEncoder(int motorID){
        switch (motorID) {
            case BACK_LEFT_MOTOR_ID:
            case FRONT_LEFT_MOTOR_ID:
                return leftEncoder;
            
            case BACK_RIGHT_MOTOR_ID:
            case FRONT_RIGHT_MOTOR_ID:
                return rightEncoder;
                
            default:
                throw new Error("Encoder ID " + motorID + " is invalid!");
        }
    }

    public double getEncoderDistance(int motorID){
        switch (motorID) {
            case BACK_LEFT_MOTOR_ID:
            case FRONT_LEFT_MOTOR_ID:
                return leftEncoder.getPosition()*NEO_DISTANCE_CALIBRATION;
            
            case BACK_RIGHT_MOTOR_ID:
            case FRONT_RIGHT_MOTOR_ID:
                return rightEncoder.getPosition()*NEO_DISTANCE_CALIBRATION;
                
            default:
                throw new Error("Encoder ID " + motorID + " is invalid!");
        }
    }

    public Double getEncoderVelocity(int motorID){
        switch (motorID) {
            case BACK_LEFT_MOTOR_ID:
            case FRONT_LEFT_MOTOR_ID:
                return leftEncoder.getVelocity()*NEO_VELOCITY_CALIBRATION;
            
            case BACK_RIGHT_MOTOR_ID:
            case FRONT_RIGHT_MOTOR_ID:
                return rightEncoder.getVelocity()*NEO_VELOCITY_CALIBRATION;
                
            default:
                throw new Error("Encoder ID " + motorID + " is invalid!");
        }
    }

    /**
     * Sets the voltages of the indvidiual motors, without PID for mecanum compatability
     * 
     * @param frontLeftSpeed the speed of the front left motor     
     * @param frontRightSpeed the speed of the front right motor
     * @param backLeftSpeed the speed of the back left motor     
     * @param backRightSpeed the speed of the back right motor
     */
    @Deprecated
    public void plugandChugDrive(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed){
        frontLeft.set(frontLeftSpeed);
        frontRight.set(frontRightSpeed);
        backLeft.set(backLeftSpeed);
        backRight.set(backRightSpeed);
    }

    public void plugAndVoltDrive(double frontLeftVoltage, double frontRightVoltage, double backLeftVoltage, double backRightVoltage){
        frontLeft.setVoltage(frontLeftVoltage);
        frontRight.setVoltage(frontRightVoltage);
        backLeft.setVoltage(backLeftVoltage);
        backRight.setVoltage(backRightVoltage);
    }

    public void plugAndVoltDrive(double leftVoltage, double rightVoltage) {
        plugAndVoltDrive(leftVoltage, rightVoltage, leftVoltage, rightVoltage);
    }

    public void setCoast(boolean a){
        if(a){
            frontLeft.setIdleMode(IdleMode.kCoast);
            frontRight.setIdleMode(IdleMode.kCoast);
            backLeft.setIdleMode(IdleMode.kCoast);
            backRight.setIdleMode(IdleMode.kCoast);
        }
        else{
            frontLeft.setIdleMode(IdleMode.kBrake);
            frontRight.setIdleMode(IdleMode.kBrake);
            backLeft.setIdleMode(IdleMode.kBrake);
            backRight.setIdleMode(IdleMode.kBrake);
        }
    }

    /**
     * Sets the speeds of the motors. Uses feedforward but not PID. (right now PID is broken)
     * 
     * @param leftSpeed the speed for the left motors in meters per second
     * @param rightSpeed the speed for the right motors in meters per second
     */
    public void plugandFFDrive(double leftSpeed, double rightSpeed) {
        
         if(Math.abs(leftSpeed - (leftEncoder.getVelocity()*NEO_VELOCITY_CALIBRATION))>1){
            //System.out.println("Left too fast");
            leftSpeed = (leftEncoder.getVelocity()*NEO_VELOCITY_CALIBRATION) + 1*Math.signum((leftSpeed - (leftEncoder.getVelocity()*NEO_VELOCITY_CALIBRATION)));
        }

        if(Math.abs(rightSpeed - (rightEncoder.getVelocity()*NEO_VELOCITY_CALIBRATION))>1){
            //System.out.println("Right too fast");
            rightSpeed = (rightEncoder.getVelocity()*NEO_VELOCITY_CALIBRATION) + 1*Math.signum((rightSpeed - (rightEncoder.getVelocity()*NEO_VELOCITY_CALIBRATION)));
        }        

        double leftVoltage = leftFeedForward.calculate(leftSpeed);
        double rightVoltage = rightFeedForward.calculate(rightSpeed);
         leftVoltage += leftPID.calculate(leftEncoder.getVelocity() * NEO_VELOCITY_CALIBRATION, leftSpeed);
         rightVoltage += rightPID.calculate(rightEncoder.getVelocity() * NEO_VELOCITY_CALIBRATION, rightSpeed);
        // Work on better PID Analyzer

        leftVoltage = MathUtil.clamp(leftVoltage, -DRIVE_MOTOR_MAX_VOLTAGE, DRIVE_MOTOR_MAX_VOLTAGE);
        rightVoltage = MathUtil.clamp(rightVoltage, -DRIVE_MOTOR_MAX_VOLTAGE, DRIVE_MOTOR_MAX_VOLTAGE);

        backLeft.follow(frontLeft);
        backRight.follow(frontRight);
        frontLeft.setVoltage(leftVoltage);
        frontRight.setVoltage(rightVoltage);
    }

    public void resetEncoders(){
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    private static double averageTwoValues(double x, double y) {
        return (x + y) / 2;
    }
    public static double getFeedForwardKs() {
        return averageTwoValues(leftKs, rightKs);
    }
    public static double getFeedForwardKv() {
        return averageTwoValues(leftKv, rightKv);
    }
    public static double getFeedForwardKa() {
        return averageTwoValues(leftKa, rightKa);
    }
}
