package frc.team5115.Commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Drivetrain;

public class DriveForward extends CommandBase{
    private Drivetrain drivetrain;
    private double dist;
    private double startRightDist;
    private double startleftDist;
    private double speed;
    private boolean doneMoving;
    private Timer timer;

    public DriveForward(Drivetrain drivetrain, double dist, double speed) {
        this.dist = dist;
        this.drivetrain = drivetrain;
        this.speed = speed;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
        startRightDist = drivetrain.getRightDistance();
        startleftDist = drivetrain.getLeftDistance();
    }

    @Override
    public void execute() {
        doneMoving = drivetrain.UpdateMoving(dist, startleftDist, startRightDist, speed);
        //System.out.println("Right Distance: " + drivetrain.getRightDistance() + "Left Distance: " + drivetrain.getRightDistance());
    }

    public void end(boolean interrupted) {
       // drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        if(timer.get() > 3.7) return true;
        return doneMoving;
    }
}