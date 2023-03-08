package frc.team5115.Classes.Hardware;

import static frc.team5115.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.MathUtil;

public class HardwareDrivetrain{

    // Competition feedforward and feedback (pid) values
    // 6 inch diameter on COMP ROBOT WITH ARM and dumbells in back
    private final double leftKs = 0.12543;
    private final double leftKv = 1.3269;
    private final double leftKa = 0.14027;
    
    private final double rightKs = 0.12477;
    private final double rightKv = 1.3587;
    private final double rightKa = 0.13818;

    private final double leftKp = 1.6455;
    private final double rightKp = 1.6220;
    private final double Ki = 0.0;
    private final double Kd = 0.0;
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

    public void PlugandVoltDrive(double frontLeftVoltage, double frontRightVoltage, double backLeftVoltage, double backRightVoltage){
        frontLeft.setVoltage(frontLeftVoltage);
        frontRight.setVoltage(frontRightVoltage);
        backLeft.setVoltage(backLeftVoltage);
        backRight.setVoltage(backRightVoltage);
    }

    /**
     * Sets the speeds of the motors. Uses feedforward but not PID. (right now PID is broken)
     * 
     * @param leftSpeed the speed for the left motors in meters per second
     * @param rightSpeed the speed for the right motors in meters per second
     */
    public void plugandFFDrive(double leftSpeed, double rightSpeed) {
        
        double leftVoltage = 3*leftFeedForward.calculate(leftSpeed);
        double rightVoltage = 3*rightFeedForward.calculate(rightSpeed);
        // leftVoltage += leftPID.calculate(leftEncoder.getVelocity() * NEO_ENCODER_CALIBRATION, leftSpeed);
        // rightVoltage += rightPID.calculate(rightEncoder.getVelocity() * NEO_ENCODER_CALIBRATION, rightSpeed);

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
}