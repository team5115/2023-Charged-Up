package frc.team5115.Classes.Software;

import frc.team5115.Constants;

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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * The drivetrain subsystem. Provides a number of high-complexity utility functions for interacting with the drivetrain.
 */
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
    private DifferentialDrivePoseEstimator poseEstimator;
    private double leftSpeed;
    private double rightSpeed;

    public static final double kD = 0.25;
    public static final double hD = 0.044;
    public static final double bA = 10;
    public static final double MaxArea = 0.1;

    public Drivetrain(PhotonVision photonVision, HardwareDrivetrain hardwareDrivetrain, NAVx nav) {
        this.photonVision = photonVision;
        throttle = new ThrottleControl(3, -3, 0.2);
        anglePID = new PIDController(0.019, 0.0001, 0.0012);
        
        drivetrain = hardwareDrivetrain;
        ramseteController = new RamseteController();
        kinematics = new DifferentialDriveKinematics(Constants.TRACKING_WIDTH_METERS);
        navx = nav;
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

	/**
	 * Stops all motors on the drivetrain.
	 */
    public void stop() {
        drivetrain.PlugAndVoltDrive(0, 0, 0, 0);
    }

	/**
	 * @return The distance the left side of the drivetrain has traveled
	 */
    public double getLeftDistance(){
        return drivetrain.getEncoderDistance(Constants.BACK_LEFT_MOTOR_ID);
    }

	/**
	 * @return The distance the right side of the drivetrain has traveled
	 */
    public double getRightDistance(){
        return drivetrain.getEncoderDistance(Constants.BACK_RIGHT_MOTOR_ID);
    }

	/**
	 * Sets the encoder values to 0.
	 */
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
     * Enable or disable throttle. set to false to make throttle.getThrottle() return 0, true for normal function
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
        drivetrain.plugAndChugDrive(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
    }

    /**
     * Drive the robot using a tank drive setup.
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
        drivetrain.plugAndFFDrive(leftSpeed, rightSpeed);
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
        System.out.println("remaining degrees: " + (rotationDegrees-angleDegrees));
        double turn = MathUtil.clamp(anglePID.calculate(rotationDegrees, angleDegrees), -0.75, 0.75);
        leftSpeed = turn;
        rightSpeed = -turn;
        
        drivetrain.plugAndFFDrive(leftSpeed, rightSpeed);
        return Math.abs(rotationDegrees-angleDegrees)<15;
        }

	/**
	 * Drives the robot to a trajectory state.
	 * @param tState - The trajectory state to drive to
	 */
    public void TankDriveToTrajectoryState(Trajectory.State tState) {
        final ChassisSpeeds adjustedSpeeds = ramseteController.calculate(getEstimatedPose(), tState);
        final DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);
        leftSpeed = wheelSpeeds.leftMetersPerSecond;
        rightSpeed = wheelSpeeds.rightMetersPerSecond;
        drivetrain.plugAndFFDrive(leftSpeed, rightSpeed);
        System.out.println("left: " + leftSpeed + " | right: " + rightSpeed);
        System.out.println(wheelSpeeds);
        System.out.println(getEstimatedPose());
    }

	/**
	 * Updates the odometry of the robot.
	 */
    public void UpdateOdometry() {
        poseEstimator.update(navx.getYawRotation2D(), getLeftDistance(), getRightDistance());

        Optional<EstimatedRobotPose> result = photonVision.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
            System.out.println("vision is really working");
        }
    }

	/**
	 * @return The estimated pose of the robot
	 */
    public Pose2d getEstimatedPose() {
        UpdateOdometry();
        return poseEstimator.getEstimatedPosition();
    }

	/**
	 * Generate a command that will make the robot follow a given trajectory.
	 * @param trajectory The trajectory to follow
	 */
    public Command getRamseteCommand(Trajectory trajectory) {
        //drivetrain.setCoast(true);
        final double MaxSpeed = 0.1; // m/s
        final double MaxAcceleration = 0.1; // m/s^2
        final SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(
            drivetrain.leftKs,
            drivetrain.leftKv,
            drivetrain.leftKa
        );
        final var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(simpleMotorFeedforward, kinematics, 10);
        
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(MaxSpeed, MaxAcceleration);
        config.setKinematics(kinematics);
        config.addConstraint(autoVoltageConstraint);
        config.setReversed(true);
    
        // An example trajectory to follow.  All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(+0, +0, new Rotation2d(+0)), 
        //     new ArrayList<Translation2d>(), 
        //     new Pose2d(+3, +1, new Rotation2d(+0)),
        //     config
        // );
    
        RamseteCommand ramseteCommand =
            new RamseteCommand(
                trajectory,
                this :: getEstimatedPose,
                new RamseteController(),
                simpleMotorFeedforward,
                kinematics,
                this :: getWheelSpeeds,
                new PIDController(0, 0, 0),
                new PIDController(0, 0, 0),
                // RamseteCommand passes volts to the callback
                drivetrain :: PlugAndVoltDrive
                );
    
        // Reset odometry to the starting pose of the trajectory.
        //resetOdometry(exampleTrajectory.getInitialPose());
    
        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> stop());
    }

	/**
	 * @return A `DifferentialDriveWheelSpeeds` object containing the current speeds of the wheels
	 */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(drivetrain.getEncoderVelocity(1), drivetrain.getEncoderVelocity(2));
    }

    public boolean UpdateMoving(double dist, double startLeftDist, double startRightDist, double speedMagnitude, double heading) {
        final double remainingLeftDistance = startLeftDist + dist - getLeftDistance();
        final double remainingRightDistance = startRightDist + dist - getLeftDistance();

        final double forward = speedMagnitude * Math.signum((remainingLeftDistance + remainingRightDistance) / 2);
        final double turn = MathUtil.clamp(anglePID.calculate(getYawDeg(), heading), -0.3, +0.3);
        leftSpeed = forward + turn;
        rightSpeed = forward - turn;
        drivetrain.plugAndFFDrive(leftSpeed, rightSpeed);

        final double tolerance = 0.05;
        return Math.abs(remainingLeftDistance) < tolerance || Math.abs(remainingRightDistance) < tolerance;
    }      

    public boolean UpdateMovingWithVision(double dist, Pose2d pose, double speedMagnitude) {
        double realdist = pose.getTranslation().getDistance(getEstimatedPose().getTranslation());
        final double speed = speedMagnitude * Math.signum(dist);
        drivetrain.plugAndFFDrive(speed, speed);
        final double tolerance = 0.1;
        return Math.abs(realdist-dist) < tolerance;
    }

	/**
	 * Resets the NAVx.
	 */
    public void resetNAVx(){
        navx.resetNAVx();
    }

	/**
	 * @return The current pitch in degrees, given by the NAVx
	 */
    public double getPitchDeg() {
        return navx.getPitchDeg();
    }

	/**
	 * @return The current yaw in degrees, given by the NAVx
	 */
    public double getYawDeg() {
        return navx.getPitchDeg();
    }

    /**
     * Drive forward at a given speed.
     * @param speed - The speed to drive at in m/s
     */
    public void autoDrive(double speed){
        drivetrain.plugAndFFDrive(speed, speed);
    }

    @Deprecated
    public void autoDriveF(){
        drivetrain.plugAndChugDrive(0.3, -0.3, 0.3, -0.3);
    }

    @Deprecated
    public void autoDriveB(){
        drivetrain.plugAndChugDrive(-0.3, 0.3, -0.3, 0.3);
    }
    /**
     * Drive backward at 1 m/s
     */
    public void autoDriveBackward(){
        drivetrain.plugAndFFDrive(-1, -1);
    }

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
        drivetrain.plugAndFFDrive(leftSpeed, rightSpeed);
        System.out.println("left speed "+ leftSpeed);
        System.out.println("right speed "+ rightSpeed);
    }

	/**
	 * @return The current x position
	 */
    public double getX(){
        return tx.getDouble(0);
    }

	/**
	 * @return The current y position
	 */
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
            leftSpeed = -(Constants.TARGET_ANGLE - yangle)*hD;
            leftSpeed = Math.max(-0.3, Math.min(0.3, leftSpeed));
            rightSpeed = leftSpeed;
            drivetrain.plugAndFFDrive(leftSpeed, rightSpeed);
        }
    }
}
