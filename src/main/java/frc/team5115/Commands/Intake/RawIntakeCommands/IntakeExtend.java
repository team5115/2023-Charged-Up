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
    double bottomSpeed = 0;
    double topSpeed = 0;


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
        //System.out.println(timer.get() + " " + intake.getTopWinchLength() + " " + intake.getBottomWinchLength());
    }

    public void end(boolean interrupted){
        System.out.println("Stopped");
    }

    public boolean isFinished() {
        if((Math.abs(intake.getBottomWinchLength()-bottomLength)<1) && (intake.getTopWinchLength()-topLength)<1){
            return true;
        }
        else innerTimer.reset();
        if(timer.get()>0.2){
            return true;
        }

        return false;
      }


}
