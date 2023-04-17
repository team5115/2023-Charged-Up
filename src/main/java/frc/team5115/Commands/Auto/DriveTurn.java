package frc.team5115.Commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Hardware.NAVx;
import frc.team5115.Classes.Software.Drivetrain;

public class DriveTurn extends CommandBase{
    private final Timer grandTimer;
    private final Drivetrain drivetrain;
    private final double angle;
    private boolean turned = false;

    public DriveTurn(Drivetrain drivetrain, double absoluteAngle) {
        this.drivetrain = drivetrain;
        angle = NAVx.clampAngle(absoluteAngle);
        grandTimer = new Timer();
    }

    @Override
    public void initialize() {
        grandTimer.reset();
        grandTimer.start();
    }

    @Override
    public void execute() {
        turned = drivetrain.TankDriveToAngle(angle);
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("finished turning");
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        // timeout if the command has been running for too long
        if (grandTimer.get() > 4) {
            System.out.println("Turning attempt timed out after 4 seconds");
            return true;
        }
        // finish if it didn't move this timestep
        return turned;
    }
}