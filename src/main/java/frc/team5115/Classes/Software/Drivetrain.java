package frc.team5115.Classes.Software;

import static frc.team5115.Constants.*;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Acessory.ThrottleControl;
import frc.team5115.Classes.Hardware.HardwareDrivetrain;
import frc.team5115.Classes.Hardware.NAVx;

public class Drivetrain extends SubsystemBase{
    
    public NetworkTable ShooterCam;
    public NetworkTableEntry ty;
    public NetworkTableEntry tx;
    public NetworkTableEntry tv;
    private final ThrottleControl throttle;
    private final PIDController anglePID;
    private final RamseteController ramseteController;
    private final DifferentialDriveKinematics kinematics;
    private final HardwareDrivetrain drivetrain;
    private final NAVx navx;
    private final PhotonVision photonVision;
    private final DifferentialDrivePoseEstimator poseEstimator;
    private double leftSpeed;
    private double rightSpeed;

    public static final double kD = 0.25;
    public static final double hD = 0.044;
    public static final double bA = 10;
    public static final double MaxArea = 0.1;

    public Drivetrain(PhotonVision photonVision) {
        this.photonVision = photonVision;
        throttle = new ThrottleControl(3, -3, 0.2);
        anglePID = new PIDController(0.0144, 0.0001, 0.0015);
        drivetrain = new HardwareDrivetrain();
        ramseteController = new RamseteController();
        kinematics = new DifferentialDriveKinematics(TRACKING_WIDTH_METERS);
        navx = new NAVx();
        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, navx.getRotation2D(), 0.0, 0.0, new Pose2d());
    }

    public void stop() {
        drivetrain.plugandFFDrive(0, 0);
    }

    public void resetEncoders() {
        drivetrain.resetEncoders();
    }

    public void toggleThrottle(){
        throttle.toggleThrottle();
    }

    public void toggleSlowMode() {
        throttle.toggleSlowMode();
    }

    @Deprecated
    public void TankDriveOld(double forward, double turn){
        if(forward>0.5){
            forward = 0.3;
        }

        else if(forward<0){
            forward = 0;
        }

        if(turn>0.5){
            turn = 0.5;
        }
        else if(turn < -0.5){
            turn = -0.5;
        }

        leftSpeed = (forward + turn);
        rightSpeed = (forward - turn);
        drivetrain.plugandChugDrive(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
    }

    /**
     * Drive the robot using a tankdrive setup.
     * @param forward is for driving forward/backward: positive is forward, negative is backward
     * @param turn is for turning right/left: positive is right, negative is left
     */
    public void TankDrive(double forward, double turn) { 
        if (throttle.getThrottleSwitched()) {
            // keep the turning from being switched by throttle switch by switching it back
            turn = -turn;
        }
        leftSpeed = (forward + turn);
        rightSpeed = (forward - turn);

        leftSpeed *= throttle.getThrottle();
        rightSpeed *= throttle.getThrottle();

        drivetrain.plugandFFDrive(leftSpeed, rightSpeed);
    }

    @Deprecated
    public void TankDriveToAngle(double angleDegrees) { 
        double rotationDegrees = navx.getNavxRotationDeg();
        System.out.println(rotationDegrees);
        double turn = MathUtil.clamp(anglePID.calculate(rotationDegrees, angleDegrees), -1, 1);
        leftSpeed = turn;
        rightSpeed = -turn;
        
        drivetrain.plugandFFDrive(leftSpeed, rightSpeed);
    }

    public void TankDriveToTrajectoryState(Trajectory.State tState) {
        ChassisSpeeds adjustedSpeeds = ramseteController.calculate(UpdateOdometry(), tState);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);
        leftSpeed = wheelSpeeds.leftMetersPerSecond;
        rightSpeed = wheelSpeeds.rightMetersPerSecond;
        drivetrain.plugandFFDrive(leftSpeed, rightSpeed);
    }

    public Pose2d UpdateOdometry() {
        poseEstimator.update(navx.getRotation2D(),
            drivetrain.getEncoder(FRONT_LEFT_MOTOR_ID).getPosition(),
            drivetrain.getEncoder(FRONT_RIGHT_MOTOR_ID).getPosition()
        );

        Optional<EstimatedRobotPose> result = photonVision.getEstimatedGlobalPose();
        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
            return poseEstimator.getEstimatedPosition();
        }
        return poseEstimator.getEstimatedPosition();
    }

    public void resetNAVx(){
        navx.resetNAVX();
    }

    /**
     * Drive forward at 1 m/s
     */
    public void autoDriveForward(){
        drivetrain.plugandFFDrive(1, 1);
    }

    @Deprecated
    public void autoDriveF(){
        drivetrain.plugandChugDrive(0.3, -0.3, 0.3, -0.3);
    }

    @Deprecated
    public void autoDriveB(){
        drivetrain.plugandChugDrive(-0.3, 0.3, -0.3, 0.3);
    }
    /**
     * Drive backward at 1 m/s
     */
    public void autoDriveBackward(){
        drivetrain.plugandFFDrive(-1, -1);
    }
    /**
     * Drive all motors at a specific voltage
     * @param percent voltage to drive at
     */

    @Deprecated
    public void AdjustAngle(){
        double xangle = 0; 
        double detector = 0;
        leftSpeed = -xangle*kD;
        if(leftSpeed > 0.3){
            leftSpeed = 0.3;
            System.out.println("capping speed");
        }
        if(leftSpeed < -0.3){
            leftSpeed = -0.3;
            System.out.println("capping speed");
        }
        if(!(detector == 1)){
            leftSpeed = 0;
            System.out.print("nothing detected");
        }

        rightSpeed = leftSpeed;
        drivetrain.plugandFFDrive(leftSpeed, rightSpeed);
        System.out.println("left speed "+ leftSpeed);
        System.out.println("right speed "+ rightSpeed);
    }

    public double getX(){
        return tx.getDouble(0);
    }

    public double getY(){
        return ty.getDouble(0);
    }

    // public double getDistanceFromHub(){
    //     double yAngle = ty.getDouble(0);
    //     d = (AUTO_HIGH_GOAL_HEIGHT - AUTO_CAMERA_HEIGHT) / tan(toRadians(yAngle + AUTO_CAMERA_ANGLE));
    //     return d;
    // }

    @Deprecated
    public void AdjustDistance(){
        double dectector = 0;
        if(dectector == 1){
            /**d = (AUTO_HIGH_GOAL_HEIGHT - AUTO_CAMERA_HEIGHT) / tan(toRadians(yangle + AUTO_CAMERA_ANGLE));
            leftSpd = (d-HUB_DISTANCE)*hD;
            rightSpd = -(d - HUB_DISTANCE)*hD;
            */
            double yangle = 0; 
            leftSpeed = -(TARGET_ANGLE - yangle)*hD;
            leftSpeed = Math.max(-0.3, Math.min(0.3, leftSpeed));
            rightSpeed = leftSpeed;
            drivetrain.plugandFFDrive(leftSpeed, rightSpeed);
        }
    }
}