package frc.team5115.Commands.Auto.DockAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Drivetrain;

public class DriveUntilDock extends CommandBase {
    Drivetrain drivetrain;
    Timer timer;
    double timeout;

    public DriveUntilDock(Drivetrain drivetrain, double timeout) {
        this.drivetrain = drivetrain;
        this.timeout = timeout;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        drivetrain.autoDriveForward();
    }

    @Override
    public boolean isFinished() {
        if (timer.get() > timeout) {
            System.out.println("Drive until dock timed out");
            return true;
        }
        if (drivetrain.getPitchDeg() > 5) {
            return true;
        }
        return false;
    }
}
