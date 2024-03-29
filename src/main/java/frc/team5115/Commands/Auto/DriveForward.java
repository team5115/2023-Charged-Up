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
    private final double timeout;

    public DriveForward(Drivetrain drivetrain, double dist, double speed) {
        this.dist = dist;
        this.drivetrain = drivetrain;
        this.speed = speed;
        timeout = Math.abs(dist / speed) + 1; // meters divided by m/s gives seconds! plus extra time
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
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        if(timer.get() > timeout) return true;
        return doneMoving;
    }
}