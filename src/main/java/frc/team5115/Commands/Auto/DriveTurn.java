package frc.team5115.Commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Drivetrain;

public class DriveTurn extends CommandBase{
    private Timer grandTimer;
    private Timer doneTimer;
    private Drivetrain drivetrain;
    private double absoluteAngle;

    public DriveTurn(Drivetrain drivetrain, double deltaAngle) {
        absoluteAngle = deltaAngle + drivetrain.getYawDeg();
        this.drivetrain = drivetrain;
        doneTimer = new Timer();
        grandTimer = new Timer();
        grandTimer.start();
    }

    @Override
    public void initialize() {
        grandTimer.reset();
    }

    @Override
    public void execute() {
        boolean turned = drivetrain.UpdateTurning(absoluteAngle);
        if (turned) {
            doneTimer.start();
        } else {
            doneTimer.reset();
        }
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("finished turning");
    }

    @Override
    public boolean isFinished() {
        // timeout if the command has been running for too long
        if (grandTimer.get() > 10) {
            System.out.println("Turning attempt timed out after 10 seconds");
            return true;
        }
        // finish if docked for more than the minimum dock time
        if (doneTimer.get() > 0.2) {
            System.out.println("Successfully turned");
            return true;
        }
        return false;
    }
}