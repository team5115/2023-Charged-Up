package frc.team5115.Commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Hardware.NAVx;
import frc.team5115.Classes.Software.Drivetrain;

public class DriveTurn extends CommandBase{
    private Timer grandTimer;
    private Timer doneTimer;
    private Drivetrain drivetrain;
    private double absoluteAngle;
    private double deltaAngle;
    private boolean turned = false;

    public DriveTurn(Drivetrain drivetrain, double deltaAngle) {
        this.drivetrain = drivetrain;
        this.deltaAngle = deltaAngle;
        doneTimer = new Timer();
        grandTimer = new Timer();
        grandTimer.start();
    }

    @Override
    public void initialize() {
        absoluteAngle = NAVx.clampAngle(deltaAngle + drivetrain.getYawDeg());
        grandTimer.reset();
        turned = false;
    }

    @Override
    public void execute() {
         turned = drivetrain.TankDriveToAngle(absoluteAngle);
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("finished turning");
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        // timeout if the command has been running for too long
        if (grandTimer.get() > 2) {
            System.out.println("Turning attempt timed out after 10 seconds");
            return true;
        }
        // finish if docked for more than the minimum dock time
        return turned;
    }
}