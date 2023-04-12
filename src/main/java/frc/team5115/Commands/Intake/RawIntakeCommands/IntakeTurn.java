package frc.team5115.Commands.Intake.RawIntakeCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Arm;

public class IntakeTurn extends CommandBase{
    private Arm intake;
    private Timer timer;
    private Timer innerTimer;
    private double angle;

    public IntakeTurn(Arm a, double angle){
        intake = a;
        this.angle = angle;
        timer = new Timer();
        timer.start();
        innerTimer = new Timer();
        innerTimer.start();
    }
    public void initialize() {
        timer.reset();
        //innerTimer.reset();
        intake.turnSetAngle(angle);
    }

    public void execute(){
        System.out.println("Arm Rotation Degrees: " + intake.getAngle()+ " can turn? " + intake.armcontrolangle);
    }

    public void end(boolean interrupted){
        System.out.println("Stopped in IntakeTurn");
    }

    public boolean isFinished() {
        
        if((Math.abs(intake.getAngle()-angle)<2)){
            if(innerTimer.get() > 0.05) return true;
        }
        else innerTimer.reset();

        return false;
      }

}
