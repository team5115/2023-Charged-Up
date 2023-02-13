package frc.team5115.Commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Drivetrain;

public class DriveTurn extends CommandBase{
    private Timer grandTimer;
    private Timer dockedTimer;
    private Drivetrain drivetrain;
    private double angle;

    public DriveTurn(Drivetrain drivetrain, double angle) {
        this.angle = angle;
        this.drivetrain = drivetrain;
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
        boolean turned = drivetrain.UpdateTurning(angle);
        if (turned) {
            dockedTimer.start();
        } else {
            dockedTimer.reset();
        }
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("finished charging");
    }

    @Override
    public boolean isFinished() {
        // timeout if the command has been running for too long
        if (grandTimer.get() > 10) {
            System.out.println("Docking attempt timed out after 10 seconds");
            return true;
        }
        // finish if docked for more than the minimum dock time
        if (dockedTimer.get() > 0.5) {
            System.out.println("Successfully docked");
            return true;
        }
        return false;
    }
}