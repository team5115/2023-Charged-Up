package frc.team5115.Commands.Auto.DockAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Drivetrain;

public class Dock extends CommandBase{
    private Timer grandTimer;
    private Timer dockedTimer;
    private Drivetrain drivetrain;
    private double encoderDistanceAtStart;

    public Dock(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        dockedTimer = new Timer();
        grandTimer = new Timer();
        grandTimer.start();
    }

    @Override
    public void initialize() {
        encoderDistanceAtStart = drivetrain.getLeftDistance();
        grandTimer.reset();
    }

    @Override
    public void execute() {
        boolean balanced = drivetrain.UpdateDocking();
        System.out.println("dist from start: " + (drivetrain.getLeftDistance() - encoderDistanceAtStart));
        if (balanced) {
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
        if (grandTimer.get() > 60) {
            System.out.println("Docking attempt timed out after 60 seconds");
            return true;
        }
        // finish if docked for more than the minimum dock time
        if (dockedTimer.get() > 3) {
            System.out.println("Successfully docked @ " + drivetrain.getPitchDeg());
            return true;
        }
        return false;
    }
}