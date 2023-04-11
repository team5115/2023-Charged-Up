package frc.team5115.Commands.Intake.RawIntakeCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Arm;

public class IntakeTurn extends CommandBase{
    private Arm arm;
    private Timer timer;
    private Timer innerTimer;
    private double angle;

    public IntakeTurn(Arm arm, double angle){
        this.arm = arm;
        this.angle = angle;
        timer = new Timer();
        timer.start();
        innerTimer = new Timer();
        innerTimer.start();
    }
    public void initialize() {
        timer.reset();
        //innerTimer.reset();
        arm.turnSetAngle(angle);
    }

    public void execute(){
        System.out.println("Arm Rotation Degrees: " + arm.getAngle());
    }

    public void end(boolean interrupted){
        System.out.println("Stopped in IntakeTurn");
    }

    public boolean isFinished() {
        
        if((Math.abs(arm.getAngle()-angle)<2)){
            if(innerTimer.get() > 0.05) return true;
        }
        else innerTimer.reset();

        return false;
      }

}
