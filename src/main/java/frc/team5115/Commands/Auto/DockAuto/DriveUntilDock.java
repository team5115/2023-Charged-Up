package frc.team5115.Commands.Auto.DockAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Drivetrain;

public class DriveUntilDock extends CommandBase {
    Drivetrain drivetrain;
    Timer timer;
    Timer innerTimer;
    double direction;
    final double timeout = 10;
    final double finishedTolerance = 5; // degrees

    public DriveUntilDock(Drivetrain drivetrain, double direction) {
        this.drivetrain = drivetrain;
        this.direction = direction;
        timer = new Timer();
        innerTimer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        innerTimer.reset();
        innerTimer.start();

        drivetrain.setThrottleEnabled(false);
    }

    @Override
    public void execute() {
        // only "works" when going backwards; direction is always 1 
        drivetrain.autoDrive(0.9 * direction);
        // System.out.println("hasn't found it yet @ " + drivetrain.getPitchDeg() + " degrees");
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(drivetrain.getPitchDeg()) > finishedTolerance) {
            System.out.println("found slope");
            innerTimer.start();
            if(innerTimer.get()  > 0.65)
            return true;
        }
        else{
            innerTimer.reset();
        }
        return false;
    }
}
