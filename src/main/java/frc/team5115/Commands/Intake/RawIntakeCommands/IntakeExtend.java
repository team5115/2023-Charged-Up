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
        /* 
        double bottomSpeed = intake.bottomWinchController.calculate(intake.getBottomWinchLength(), bottomLength);
        double topSpeed = intake.topWinchController.calculate(intake.getTopWinchLength(), topLength);
        if((intake.getTopWinchLength() - intake.getBottomWinchLength()+3)>0.5){
            System.out.print("Top too far ahead & ");
                if(topSpeed< 0 && bottomSpeed<0){
                    System.out.println("Bottom Stopped");
                    intake.setBottomWinch(0);
                    intake.setTopWinch(topSpeed);
                }
                else if(topSpeed<0 && bottomSpeed>0){
                    System.out.println("Neither Stopped");
                    intake.setTopWinch(topSpeed);
                    intake.setBottomWinch(bottomSpeed);
                }
                else if(topSpeed>0  && bottomSpeed>0){
                    System.out.println("top Stopped");
                    intake.setTopWinch(0);
                    intake.setBottomWinch(bottomSpeed);
                }
                else{
                    System.out.println("none Stopped");
                    intake.setTopWinch(topSpeed);
                    intake.setBottomWinch(bottomSpeed);
                }
            }
            else if(intake.getBottomWinchLength() - intake.getTopWinchLength()-3>0.5){
                System.out.print("Bottom too far ahead");
                if(topSpeed< 0 && bottomSpeed<0){
                    System.out.println("Top Stopped");
                    intake.setTopWinch(0);
                    intake.setBottomWinch(bottomSpeed);
                }
                else if(topSpeed>0 && bottomSpeed<0){
                    System.out.println("Neither Stopped");
                    intake.setTopWinch(topSpeed);
                    intake.setBottomWinch(bottomSpeed);
                }
                else if(topSpeed>0  && bottomSpeed>0){
                    System.out.println("Bottom Stopped");
                    intake.setTopWinch(topSpeed);
                    intake.setBottomWinch(0);
                }
                else{
                    System.out.println("none Stopped");
                    intake.setTopWinch(topSpeed);
                    intake.setBottomWinch(bottomSpeed);
                }
            }
        else{
            System.out.println("Everything fine");
        }
        //System.out.println(intake.getBottomWinchLength() + " " + intake.getTopWinchLength());
        */
    }

    public void end(boolean interrupted){
        System.out.println("Stopped");
    }

    public boolean isFinished() {
        if((Math.abs(intake.getBottomWinchLength()-bottomLength)<1) && (intake.getTopWinchLength()-topLength)<1){
            if(innerTimer.get() > 0.2) return true;
        }
        else innerTimer.reset();

        if(timer.get()>2){
            return true;
        }
        return false;
      }


}
