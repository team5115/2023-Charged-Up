package frc.team5115.Classes.Hardware;

import static frc.team5115.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.controller.*;
import frc.team5115.Classes.Software.Arm;
import edu.wpi.first.math.MathUtil;

/**
 * The drivetrain hardware subsystem. Provides methods to interact with the actual hardware of the drivetrain.
 */
public class HardwareDrivetrain{
    // Competition feedforward values - 6 inch diameter on KITT comp robot with arm and ballasts
    public final double leftKs = 0.0378;
    public final double leftKv = 2.7479;
    public final double leftKa = 0.32825;
    
    private final double rightKs = 0.0528;
    private final double rightKv = 2.8399;
    private final double rightKa = 0.26071;
    
    // private final double combinedkP = 3.4349;
    // END of comp robot values

    // Testbed feedforward values - 6 inch diameter on KATT testbed
    // private final double leftKs = 0.090949;
    // private final double leftKv = 2.783;
    // private final double leftKa = 0.16477;
    
    // private final double rightKs = 0.099706;
    // private final double rightKv = 2.8314;
    // private final double rightKa = 0.14565;
    // END of testbed values
    

    private final double leftKp = 0.2;
    private final double rightKp = 0.2;
    private final double Ki = 0.1;
    private final double Kd = 0.1;
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

	/**
	 * `HardwareDrivetrain` constructor.
	 * @param arm - The arm subsystem to use
	 */
    public HardwareDrivetrain(Arm arm){
        resetEncoders();
        frontRight.setInverted(true);
    }

    /**
     * @param motorID - The ID of the motor to get an encoder of.
     * @return A reference to an encoder matching the id
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
	 * @param motorID - The ID of the motor to get the encoder distance of
	 * @return The distance traveled by the motor of the given ID
	 */
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

	/**
	 * @param motorID - The ID of the motor to get the encoder velocity of
	 * @return The velocity of the motor of the given ID
	 */
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
     * Sets the voltages of the individual motors, without PID for mecanum compatibility
     * @param frontLeftSpeed - The speed of the front left motor     
     * @param frontRightSpeed - The speed of the front right motor
     * @param backLeftSpeed - The speed of the back left motor     
     * @param backRightSpeed - The speed of the back right motor
     */
    @Deprecated
    public void plugAndChugDrive(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed){
        frontLeft.set(frontLeftSpeed);
        frontRight.set(frontRightSpeed);
        backLeft.set(backLeftSpeed);
        backRight.set(backRightSpeed);
    }

    public void PlugAndVoltDrive(double frontLeftVoltage, double frontRightVoltage, double backLeftVoltage, double backRightVoltage){
        frontLeft.setVoltage(frontLeftVoltage);
        frontRight.setVoltage(frontRightVoltage);
        backLeft.setVoltage(backLeftVoltage);
        backRight.setVoltage(backRightVoltage);
    }

	// ^^^
	// What the ****
	// Why do these have the same names?
	// ˅˅˅

    public void PlugAndVoltDrive(double leftVoltage, double rightVoltage) {
        PlugAndVoltDrive(leftVoltage, rightVoltage, leftVoltage, rightVoltage);
    }

    /**
     * Sets the speeds of the motors. Uses feedforward but not PID. (right now PID is broken)
     * 
     * @param leftSpeed the speed for the left motors in meters per second
     * @param rightSpeed the speed for the right motors in meters per second
     */
    public void plugAndFFDrive(double leftSpeed, double rightSpeed) {
        final double accelerationLimit = getAccelerationLimit(); // can't bother figuring the units, but it's not m/s^2
        final double currentLeftVelocity = getLeftVelocity();
        final double currentRightVelocity = getRightVelocity();

        // limit left acceleration
        if(Math.abs(leftSpeed - currentLeftVelocity) > accelerationLimit) {
            leftSpeed = currentLeftVelocity + accelerationLimit * Math.signum(leftSpeed - currentLeftVelocity);
        }
        // limit right acceleration
        if(Math.abs(rightSpeed - currentRightVelocity) > accelerationLimit) {
            rightSpeed = currentRightVelocity + accelerationLimit * Math.signum(rightSpeed - currentRightVelocity);
        }

        double leftVoltage = leftFeedForward.calculate(leftSpeed);
        double rightVoltage = rightFeedForward.calculate(rightSpeed);
        leftVoltage += leftPID.calculate(currentLeftVelocity, leftSpeed);
        rightVoltage += rightPID.calculate(currentRightVelocity, rightSpeed);
        // Work on better PID Analyzer

        leftVoltage = MathUtil.clamp(leftVoltage, -DRIVE_MOTOR_MAX_VOLTAGE, DRIVE_MOTOR_MAX_VOLTAGE);
        rightVoltage = MathUtil.clamp(rightVoltage, -DRIVE_MOTOR_MAX_VOLTAGE, DRIVE_MOTOR_MAX_VOLTAGE);

        if(Math.abs(leftSpeed) < 0.05) leftVoltage = 0;
        if(Math.abs(rightSpeed) < 0.05) rightVoltage = 0;

        backLeft.follow(frontLeft);
        backRight.follow(frontRight);
        frontLeft.setVoltage(leftVoltage);
        frontRight.setVoltage(rightVoltage);
    }

    private double getAccelerationLimit() {
        return 1.5;
        // final double angle = arm.getAngle();
        // final double length = (arm.getBottomWinchLength() + arm.getTopWinchLength()) / 2;
        // final double angleConstant = 0.01;
        // final double lengthConstant = 0.017;
        // return 1.0 / (((angle + 90.0) * angleConstant) + (length * lengthConstant) + 1.0) + 0.5;
    }

    public void resetEncoders(){
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public double getLeftVelocity() {
        return leftEncoder.getVelocity() * NEO_VELOCITY_CALIBRATION;
    }

    public double getRightVelocity() {
        return rightEncoder.getVelocity() * NEO_VELOCITY_CALIBRATION;
    }
}
