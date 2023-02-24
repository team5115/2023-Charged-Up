package frc.team5115.Commands.Intake.RawIntakeCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Arm;

public class IntakeExtend extends CommandBase{
    private Arm intake;
    private Timer timer;
    private Timer innerTimer;
    private double topLength;
    private double bottomLength;


    public IntakeExtend(Arm a, double topLength, double bottomLength){
        intake = a;
        this.topLength = topLength;
        this.bottomLength = bottomLength;
        timer = new Timer();
        timer.start();
        innerTimer = new Timer();
        innerTimer.start();
    }
    public void initialize() {
        timer.reset();
        innerTimer.reset();
        intake.topWinchSetLength(topLength);
        intake.bottomWinchSetLength(bottomLength);
    }

    public void execute(){
        //System.out.println(intake.getBottomWinchLength() + " " + intake.getTopWinchLength());
    }

    public void end(boolean interrupted){
        System.out.println("Stopped");
    }

    public boolean isFinished() {
        if((Math.abs(intake.getBottomWinchLength()-bottomLength)<0.1) && (intake.getTopWinchLength()-topLength)<0.1){
            if(innerTimer.get() > 0.2) return true;
        }
        else innerTimer.reset();

        if(timer.get()>2){
            return true;
        }
        return false;
      }


}
