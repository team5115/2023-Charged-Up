package frc.team5115.Commands.Auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Drivetrain;

public class FollowTrajectory extends CommandBase {
    private final Drivetrain drivetrain;
    private final Trajectory trajectory;
    private final Timer timer;
    private final double MaxSpeed = 0.1; // m/s
    private final double MaxAcceleration = 0.1; // m/s^2

    public FollowTrajectory(Drivetrain drivetrain, double x, double y, double theta, ArrayList<Translation2d> interiorWaypoints) {
        this.drivetrain = drivetrain;
        trajectory = generateTrajectory(interiorWaypoints, x, y, theta);
        timer = new Timer();
        timer.start();
    }

    public FollowTrajectory(Drivetrain drivetrain, Pose2d endPose, ArrayList<Translation2d> interiorWaypoints) {
        this.drivetrain = drivetrain;
        trajectory = generateTrajectory(interiorWaypoints, endPose);
        timer = new Timer();
        timer.start();
    }


    public FollowTrajectory(Drivetrain drivetrain, double x, double y, double theta) {
        this.drivetrain = drivetrain;
        trajectory = generateTrajectory(x, y, theta);
        timer = new Timer();
        timer.start();
    }

    public FollowTrajectory(Drivetrain drivetrain, Pose2d endPose) {
        this.drivetrain = drivetrain;
        trajectory = generateTrajectory(endPose);
        timer = new Timer();
        timer.start();
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        drivetrain.TankDriveToTrajectoryState(trajectory.sample(timer.get()));
    }
    
    @Override
    public void end(boolean interrupted){
        drivetrain.stop();
        if (interrupted) System.out.println("Trajectory following was INTERRUPTED");
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= trajectory.getTotalTimeSeconds();
    }
    
    /**
     * Create a trajectory that will drive the robot from where it is along a hard-coded path to an end position.
     * @param interiorWaypoints an ArrayList of Translation2d waypoints that the robot has to pass.
     * @param x the final x position the robot will go to.
     * @param y the final y position the robot will go to.
     * @param theta the final orentation of the robot.
     * 
     * @return a trajectory that starts at current robot position and follow hard-coded path
     * Example for how to use Interior Waypoints: 
     * ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
     * interiorWaypoints.add(new Translation2d(x, y)); 
     * Be sure to check that your taking in the x and y coordinate and not the distance and angle by checking the Javadocs of the method.
     */
    private Trajectory generateTrajectory(ArrayList<Translation2d> interiorWaypoints, double x, double y, double theta) {
        return generateTrajectory(interiorWaypoints, new Pose2d(x, y, Rotation2d.fromDegrees(theta)));
    }

    /**
     * Create a trajectory that will drive the robot from where it is along a hard-coded path to an end position.
     * @param x the final x position the robot will go to.
     * @param y the final y position the robot will go to.
     * @param theta the final orentation of the robot.
     * 
     * @return a trajectory that starts at current robot position and follow hard-coded path
     */
    private Trajectory generateTrajectory(double x, double y, double theta) {
        return generateTrajectory(new Pose2d(x, y, Rotation2d.fromDegrees(theta)));
    }

    /**
     * Create a trajectory that will drive the robot from where it is along a hard-coded path to an end position.
     * @param endPose the Pose2d object that contains the final x and y position, and orientation of the robot.
     * @return a trajectory that starts at current robot position and follow hard-coded path
     */
    private Trajectory generateTrajectory(Pose2d endPose) {
        return generateTrajectory(new ArrayList<Translation2d>(), endPose);
    }

    /**
     * Create a trajectory that will drive the robot from where it is along a hard-coded path to an end position.
     * @param interiorWaypoints an ArrayList of Translation2d waypoints that the robot has to pass.
     * @param endPose the Pose2d object that contains the final x and y position, and orientation of the robot.
     * 
     * @return a trajectory that starts at current robot position and follow hard-coded path
     * Example for how to use Interior Waypoints: 
     * ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
     * interiorWaypoints.add(new Translation2d(x, y)); 
     * Be sure to check that your taking in the x and y coordinate and not the distance and angle by checking the Javadocs of the method.
     */
    private Trajectory generateTrajectory(ArrayList<Translation2d> interiorWaypoints, Pose2d endPose) {
        TrajectoryConfig config = new TrajectoryConfig(MaxSpeed, MaxAcceleration);
        config.setReversed(true);
        return TrajectoryGenerator.generateTrajectory(drivetrain.getEstimatedPose(), interiorWaypoints, endPose, config);
    }
}