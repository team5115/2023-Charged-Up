package frc.team5115.Commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Drivetrain;

public class DriveForward extends CommandBase{
    private Drivetrain drivetrain;
    private double dist;
    private double startRightDist;
    private double startleftDist;
    private double speed;
    private boolean doneMoving;

    public DriveForward(Drivetrain drivetrain, double dist, double speed) {
        this.dist = dist;
        this.drivetrain = drivetrain;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        startRightDist = drivetrain.getRightDistance();
        startleftDist = drivetrain.getLeftDistance();
    }

    @Override
    public void execute() {
        doneMoving = drivetrain.UpdateMoving(dist, startleftDist, startRightDist, speed);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return doneMoving;
    }
}