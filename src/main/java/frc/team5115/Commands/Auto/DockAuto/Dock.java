package frc.team5115.Commands.Auto.DockAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Drivetrain;

public class Dock extends CommandBase{
    private Timer grandTimer;
    private Timer dockedTimer;
    private Drivetrain drivetrain;

    public Dock(Drivetrain drivetrain) {
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
        boolean balanced = drivetrain.UpdateDocking();
        System.out.println("balanced? " + balanced);
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
        if (grandTimer.get() > 10) {
            System.out.println("Docking attempt timed out after 10 seconds");
            return true;
        }
        // finish if docked for more than the minimum dock time
        if (dockedTimer.get() > 2) {
            System.out.println("Successfully docked");
            return true;
        }
        return false;
    }
}