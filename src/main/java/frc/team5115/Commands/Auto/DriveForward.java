package frc.team5115.Commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Drivetrain;

public class DriveForward extends CommandBase{
    private Timer grandTimer;
    private Timer dockedTimer;
    private Drivetrain drivetrain;
    private double dist;
    private double startRightDist;
    private double startleftDist;

    public DriveForward(Drivetrain drivetrain, double dist) {
        this.dist = dist;
        this.drivetrain = drivetrain;
        startRightDist = drivetrain.getRightDistance();
        startleftDist = drivetrain.getLeftDistance();
        dockedTimer = new Timer();
        grandTimer = new Timer();
        grandTimer.start();
    }

    @Override
    public void initialize() {
        grandTimer.reset();
    }

    @Override
    public void execute() {
        boolean moved = drivetrain.UpdateMoving(dist, startleftDist, startRightDist);
        if (moved) {
            dockedTimer.start();
        } else {
            dockedTimer.reset();
        }
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("finished moving");
    }

    @Override
    public boolean isFinished() {
        // timeout if the command has been running for too long
        if (grandTimer.get() > 10) {
            System.out.println("Moving attempt timed out after 10 s");
            return true;
        }
        // finish if docked for more than the minimum dock time
        if (dockedTimer.get() > 0.5) {
            System.out.println("Successfully moved");
            return true;
        }
        return false;
    }
}