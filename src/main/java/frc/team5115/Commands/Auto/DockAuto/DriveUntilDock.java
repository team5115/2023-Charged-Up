package frc.team5115.Commands.Auto.DockAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Drivetrain;

public class DriveUntilDock extends CommandBase {
    Drivetrain drivetrain;
    Timer timer;
    double direction;
    final double timeout = 5;

    public DriveUntilDock(Drivetrain drivetrain, double direction) {
        this.drivetrain = drivetrain;
        this.direction = direction;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        drivetrain.setThrottleEnabled(false);
    }

    @Override
    public void execute() {
        drivetrain.autoDrive(0.2 * direction);
        // System.out.println("hasn't found it yet @ " + drivetrain.getPitchDeg() + " degrees");
    }

    @Override
    public boolean isFinished() {
        if (timer.get() > timeout) {
            System.out.println("Drive until dock timed out");
            return true;
        }
        if (Math.abs(drivetrain.getPitchDeg()) > 12) {
            System.out.println("found slope");
            return true;
        }
        return false;
    }
}
