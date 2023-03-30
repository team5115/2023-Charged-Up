package frc.team5115.Classes.Software;

import static frc.team5115.Constants.*;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Classes.Acessory.ThrottleControl;
import frc.team5115.Classes.Hardware.HardwareDrivetrain;
import frc.team5115.Classes.Hardware.NAVx;
import edu.wpi.first.math.VecBuilder;

public class Drivetrain extends SubsystemBase{
    
    public NetworkTable ShooterCam;
    public NetworkTableEntry ty;
    public NetworkTableEntry tx;
    public NetworkTableEntry tv;
    private final ThrottleControl throttle;
    private final PIDController anglePID;
    private final PIDController movingPID;
    private final RamseteController ramseteController;
    private final DifferentialDriveKinematics kinematics;
    private final HardwareDrivetrain drivetrain;
    private final NAVx navx;
    private final PhotonVision photonVision;
    private DifferentialDrivePoseEstimator poseEstimator;
    private double leftSpeed;
    private double rightSpeed;

    public static final double kD = 0.25;
    public static final double hD = 0.044;
    public static final double bA = 10;
    public static final double MaxArea = 0.1;

    public Drivetrain(PhotonVision photonVision, Arm arm) {
        this.photonVision = photonVision;
        throttle = new ThrottleControl(3, -3, 0.2);
        anglePID = new PIDController(0.013, 0.0001, 0.0015);
        
        movingPID = new PIDController(0.01, 0, 0);
        drivetrain = new HardwareDrivetrain(arm);
        ramseteController = new RamseteController();
        kinematics = new DifferentialDriveKinematics(TRACKING_WIDTH_METERS);
        navx = new NAVx();
    }

    public void init() {
        // poseEstimator = new DifferentialDrivePoseEstimator(
        //     kinematics, navx.getYawRotation2D(), 0.0, 0.0,
        //     new Pose2d(FieldConstants.startX, FieldConstants.startY, FieldConstants.startAngle), 
        //     VecBuilder.fill(1, 1, 1),
        //     VecBuilder.fill(0, 0, 0)
        // );

        poseEstimator = new DifferentialDrivePoseEstimator(
            kinematics, navx.getYawRotation2D(), getLeftDistance(), getRightDistance(), new Pose2d(), VecBuilder.fill(1, 1, 1), VecBuilder.fill(0, 0, 0)
        );
        System.out.println("Angle from navx" + navx.getYawDeg()
        );
    }

    public void stop() {
        drivetrain.PlugandVoltDrive(0, 0, 0, 0);
    }

    public double getLeftDistance(){
        return drivetrain.getEncoderDistance(BACK_LEFT_MOTOR_ID);
    }

    public double getRightDistance(){
        return drivetrain.getEncoderDistance(BACK_RIGHT_MOTOR_ID);
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

    /**
     * enable or disable throttle. set to false to make throttle.getThrottle() return 0, true for normal function
     * @param enable true to allow stuff using throttle to move, false will just make getThrottle return 0
     */    
    public void setThrottleEnabled(boolean enable) {
        throttle.setThrottleEnabled(enable);
    }

    @Deprecated
    public void TankDriveOld(double forward, double turn){

        leftSpeed = (forward + turn);
        rightSpeed = (forward - turn);
        
        double[] v = normalizeVector(leftSpeed, rightSpeed);
        leftSpeed = v[0];
        rightSpeed = v[1];

        //System.out.println(leftSpeed*12);
        drivetrain.plugandChugDrive(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
    }

    /**
     * Drive the robot using a tankdrive setup.
     * @param forward is for driving forward/backward: positive is forward, negative is backward
     * @param turn is for turning right/left: positive is right, negative is left
     */
    public void TankDrive(double forward, double turn) { 
        turn *= 0.7;

        leftSpeed = (forward + turn);
        rightSpeed = (forward - turn);
        
        double[] v = normalizeVector(leftSpeed, rightSpeed);
        leftSpeed = v[0];
        rightSpeed = v[1];

        leftSpeed *= throttle.getThrottle();
        rightSpeed *= throttle.getThrottle();
        drivetrain.plugandFFDrive(leftSpeed, rightSpeed);
    }

    private double[] normalizeVector(double x, double y) {
        if(Math.abs(x) > 1){
            x = x/Math.abs(x);
            y = y/Math.abs(x);
        }
        else if (Math.abs(y) > 1){
            y = y/Math.abs(y);
            x = x/Math.abs(y);
        }
        return new double[] {x, y};
    }
    
    public boolean TankDriveToAngle(double angleDegrees) { 
        double rotationDegrees = navx.getYawDeg();
        System.out.println(rotationDegrees);
        double turn = MathUtil.clamp(anglePID.calculate(rotationDegrees, angleDegrees), -1, 1);
        leftSpeed = turn;
        rightSpeed = -turn;
        
        drivetrain.plugandFFDrive(leftSpeed, rightSpeed);
        return Math.abs(rotationDegrees-angleDegrees)<5;
    }

    public void TankDriveToTrajectoryState(Trajectory.State tState) {
        final ChassisSpeeds adjustedSpeeds = ramseteController.calculate(getEstimatedPose(), tState);
        final DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);
        leftSpeed = wheelSpeeds.leftMetersPerSecond;
        rightSpeed = wheelSpeeds.rightMetersPerSecond;
        drivetrain.plugandFFDrive(leftSpeed, rightSpeed);
        System.out.println("left: " + leftSpeed + " | right: " + rightSpeed);
        System.out.println(wheelSpeeds);
        System.out.println(getEstimatedPose());
    }

    public void UpdateOdometry() {
        poseEstimator.update(navx.getYawRotation2D(), getLeftDistance(), getRightDistance());

        Optional<EstimatedRobotPose> result = photonVision.getEstimatedGlobalPose();
        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            System.out.println("its really working");
            poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
        }
    }

    public Pose2d getEstimatedPose() {
        UpdateOdometry();
        return poseEstimator.getEstimatedPosition();
    }

    public boolean UpdateMoving(double dist, double startLeftDist, double startRightDist, double speedMagnitude) {
        final double remainingLeftDistance = startLeftDist + dist - getLeftDistance();
        final double remainingRightDistance = startRightDist + dist - getLeftDistance();

        final double speed = speedMagnitude * Math.signum((remainingLeftDistance + remainingRightDistance) / 2);
        drivetrain.plugandFFDrive(speed, speed);

        final double tolerance = 0.05;
        return Math.abs(remainingLeftDistance) < tolerance || Math.abs(remainingRightDistance) < tolerance;
    }      

    public boolean UpdateMovingWithVision(double dist, Pose2d pose, double speedMagnitude) {
        double realdist = pose.getTranslation().getDistance(getEstimatedPose().getTranslation());
        final double speed = speedMagnitude * Math.signum(dist);
        drivetrain.plugandFFDrive(speed, speed);
        final double tolerance = 0.1;
        return Math.abs(realdist-dist) < tolerance;
    }

    public void resetNAVx(){
        navx.resetNAVx();
    }

    public double getPitchDeg() {
        return navx.getPitchDeg();
    }

    public double getYawDeg() {
        return navx.getPitchDeg();
    }

    /**
     * Drive forward at speed m/s
     * @param speed
     */
    public void autoDrive(double speed){
        drivetrain.plugandFFDrive(speed, speed);
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