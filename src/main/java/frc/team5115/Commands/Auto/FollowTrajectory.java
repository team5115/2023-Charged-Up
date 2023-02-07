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

    public FollowTrajectory(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        trajectory = generateTrajectory();
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
    
    private Trajectory generateTrajectory() {
        TrajectoryConfig config = new TrajectoryConfig(3, 3);
        Pose2d endPose = new Pose2d(10, 2, Rotation2d.fromDegrees(0));
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(3, 2.5));
        interiorWaypoints.add(new Translation2d(5, 2));
        interiorWaypoints.add(new Translation2d(7, 1.5));
    
        return TrajectoryGenerator.generateTrajectory(drivetrain.UpdateOdometry(), interiorWaypoints, endPose, config);
    }
}