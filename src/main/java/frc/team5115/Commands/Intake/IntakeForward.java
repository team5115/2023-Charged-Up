package frc.team5115.Commands.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.IntakeMotor;

public class IntakeForward extends CommandBase{
    public IntakeMotor intake;
    public Timer timer; 

    public IntakeForward(IntakeMotor a){
        intake = a;
        timer = new Timer();
        timer.start();
    }
    public void initialize() {
        //intake.forwardIntake();
        timer.reset();
    }

    public void execute(){
        System.out.println(intake.getEncoder());
    }

    public void end(boolean interrupted){
        intake.stop();
        System.out.println("Stopped 1");
    }

    public boolean isFinished() {
        if(intake.getEncoder()<-13.5){
            return true;
        }

        if(timer.get()>2){
            return true;
        }

        return false;
      }


}

