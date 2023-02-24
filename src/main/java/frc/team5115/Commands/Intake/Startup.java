package frc.team5115.Commands.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Classes.Hardware.*;

public class Startup extends CommandBase{
    private Arm intake;
    private HardwareArm hardwareIntake;
    private HardwareIntake claw;
    private Timer timer;
    private Timer innerTimer;

    public Startup(Arm a, HardwareArm b, HardwareIntake claw){
        intake = a;
        hardwareIntake = b;
        this.claw = claw;
        timer = new Timer();
        timer.start();
        innerTimer = new Timer();
        innerTimer.start();
    }
    public void initialize() {
        timer.reset();
        intake.zeroArm();
        claw.open();
        hardwareIntake.setBottomWinch(-0.1);
        hardwareIntake.setTopWinch(-0.13);

    }

    public void execute(){
    }

    public void end(boolean interrupted){
        System.out.println("Stopped");
        intake.zeroArm();
        intake.stop();
        intake.armcontrol = true;
    }

    public boolean isFinished() {
        if(timer.get()>0.5){
            if(Math.abs(hardwareIntake.getBottomVelocity())<1 || Math.abs(hardwareIntake.getTopVelocity())<1){
            return true;
        }
    }
        /* 
        if((Math.abs(intake.getTurnDeg()-angle)<0.1)){
            if(innerTimer.get() > 0.5) return true;
        }
        else innerTimer.reset();

        */
        return false;
      }

}
