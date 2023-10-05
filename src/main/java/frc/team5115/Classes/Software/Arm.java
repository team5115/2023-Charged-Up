package frc.team5115.Classes.Software;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Hardware.HardwareArm;
import frc.team5115.Classes.Hardware.HardwareIntake;
import edu.wpi.first.math.controller.PIDController;

/**
 * The arm subsystem. Provides methods for controlling and getting information about the arm.
 */
public class Arm extends SubsystemBase{
    private static final double turnSlower = 0.3;
    private static final double extendSlower = 0.2;

    private static final double TURN_PID_TOLERANCE = 0;
    private static final double TURN_PID_KP = 0.04*turnSlower;
    private static final double TURN_PID_KI = 0.0*turnSlower;
    private static final double TURN_PID_KD = 0.0004*turnSlower;
    
    private HardwareArm hardwareArm;
    public HardwareIntake h;
    private double topLength = 0;
    private double bottomLength = 0;
    private double angle = -90;

    private final double topKp = 0.113*extendSlower;
    private final double bottomKp = 0.115*extendSlower;


    private PIDController turnController = new PIDController(TURN_PID_KP, TURN_PID_KI, TURN_PID_KD);
    public PIDController topWinchController = new PIDController(topKp, 0, 0);
    public PIDController bottomWinchController = new PIDController(bottomKp, 0, 0);

    public boolean armcontrol = true;
    public boolean armcontrolangle = false;

	/**
	 * `Arm` constructor.
	 * @param hardwareArm - The arm hardware subsystem to be used
	 * @param hardwareIntake - The intake hardware subsystem to be used
	 */
    public Arm(HardwareIntake hardwareIntake, HardwareArm hardwareArm){
        this.hardwareArm = hardwareArm;
        h =hardwareIntake;
        zeroArm();
        hardwareArm.setEncoders(topLength, -90);
        turnController.setTolerance(TURN_PID_TOLERANCE);
    }

    public void setTopPID(double kP){
        topWinchController.setP(kP);
    }

    public void setBottomPID(double kP){
        bottomWinchController.setP(kP);
    }

    public void stepBottomPID(){
        setBottomPID(bottomWinchController.getP()*1.01);
    }

    public void slowBottomPID(){
        setBottomPID(bottomWinchController.getP()*0.99);
    }

    public void stepTopPID(){
        setTopPID(topWinchController.getP()*1.01);
    }

    public void slowTopPID(){
        setTopPID(topWinchController.getP()*0.99);
    }

    public void resetPID(){
        setTopPID(topKp);
        setBottomPID(bottomKp);
    }

	/**
	 * Extends or retracts the arm to the specified length.
	 * @param length - The target arm length
	 */
    public void setLength(double length){
        topLength = length;
        bottomLength = length;
    }

    public void topWinchSetLength(double length){
        this.topLength = length;
    }

    public void bottomWinchSetLength(double length){
        this.bottomLength = length;
    }

    public void turnSetAngle(double angle){
        this.angle = angle;
    }

    public void turnUp() {
        angle += 9*0.02;
    }

    public void turnDown() {
        angle -= 9*0.02;
    }

    public void topMoveIn(){
        topLength += 10*0.02;
    }

    public void topMoveOut(){
        topLength -= 10*0.02;
    }

    public void bottomMoveIn(){
        bottomLength += 10*0.02;
    }

    public void bottomMoveOut(){
        bottomLength -= 10*0.02;
    }

	/**
	 * Disable brake mode on the arm's motors.
	 */
    public void disableBrake(){
        hardwareArm.disableBrake();
    }

	/**
	 * Enable brake mode on the arm's motors.
	 */
    public void enableBrake(){
        hardwareArm.enableBrake();
    }

    public void updateController(){
        if(bottomLength>5 || topLength>5){
            hardwareArm.FF = false;
        }
        else{
            hardwareArm.FF = true;
        }

        // final double delta = angle - intake.getArmDeg();
        final double pidOutput = turnController.calculate(hardwareArm.getArmDeg(), angle);
        
        if (!turnController.atSetpoint()) {
            hardwareArm.setTurn(pidOutput);
        }

        if(armcontrol){
            
            double topSpeed = topWinchController.calculate(hardwareArm.getTopWinchLength(), topLength);
            double bottomSpeed = bottomWinchController.calculate(hardwareArm.getBottomWinchLength(), bottomLength);
            hardwareArm.setTopWinch(topSpeed);
            hardwareArm.setBottomWinch(bottomSpeed);
        }
    }


    public double getTopWinchLength(){
        return hardwareArm.getTopWinchLength();
    }

    public double getBottomWinchLength(){
        return hardwareArm.getBottomWinchLength();
    }

    public double getBottomWinchVelocity() {
        return hardwareArm.getBottomVelocity();
    }

    public double getTopWinchVelocity() {
        return hardwareArm.getTopVelocity();
    }

    public void zeroArm(){
        hardwareArm.setEncoders(0, -100.0);
    }

    public void zeroLength(double angle){
        hardwareArm.setEncoders(0, angle);
    }

    public boolean getFault(CANSparkMax.FaultID f){
        return hardwareArm.getFault(f);
    }

    public void stop(){
        hardwareArm.stop();
    }

    public double getAngle() {
        return hardwareArm.getArmDeg();
    }

    public void setTopWinchSpeed(double speed) {
        hardwareArm.setTopWinch(speed);
    }

    public void setBottomWinchSpeed(double speed) {
        hardwareArm.setBottomWinch(speed);
    }
}
