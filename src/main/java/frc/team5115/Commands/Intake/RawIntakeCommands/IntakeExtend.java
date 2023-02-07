package frc.team5115.Commands.Intake.RawIntakeCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.IntakeMotor;

public class IntakeExtend extends CommandBase{
    private IntakeMotor intake;
    private Timer timer;
    private Timer innerTimer;
    private double length;

    public IntakeExtend(IntakeMotor a, double length){
        intake = a;
        this.length = length;
        timer = new Timer();
        timer.start();
        innerTimer = new Timer();
        innerTimer.start();
    }
    public void initialize() {
        timer.reset();
        innerTimer.reset();
    }

    public void execute(){
        intake.topWinchController(length);
        intake.bottomWinchController(length);
        System.out.println(intake.getBottomWinchLength() + " " + intake.getTopWinchLength());
    }

    public void end(boolean interrupted){
        intake.stop();
        System.out.println("Stopped");
    }

    public boolean isFinished() {
        if((Math.abs(intake.getBottomWinchLength()-length)<0.1) && (intake.getTopWinchLength()-length)<0.1){
            if(innerTimer.get() > 1) return true;
        }
        else innerTimer.reset();

        if(timer.get()>2){
            return true;
        }
        return false;
      }


}
