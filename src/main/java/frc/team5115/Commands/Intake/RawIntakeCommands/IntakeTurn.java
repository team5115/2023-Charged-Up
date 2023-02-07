package frc.team5115.Commands.Intake.RawIntakeCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.IntakeMotor;

public class IntakeTurn extends CommandBase{
    private IntakeMotor intake;
    private Timer timer;
    private Timer innerTimer;
    private double angle;

    public IntakeTurn(IntakeMotor a, double angle){
        intake = a;
        this.angle = angle;
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
        intake.turnController(angle);
        System.out.println(intake.getTurnDeg());
    }

    public void end(boolean interrupted){
        intake.stop();
        System.out.println("Stopped");
    }

    public boolean isFinished() {
        if((Math.abs(intake.getTurnDeg()-angle)<0.1)){
            if(innerTimer.get() > 1) return true;
        }
        else innerTimer.reset();

        if(timer.get()>2){
            return true;
        }
        return false;
      }


}
