package frc.team5115.Commands.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Classes.Hardware.*;

public class Startup_Intake extends CommandBase{
    private Arm arm;
    private HardwareArm hardwareArm;
    private HardwareIntake claw;
    private Timer timer;
    private Timer innerTimer;

    public Startup_Intake(Arm arm, HardwareArm hardwareArm, HardwareIntake claw){
        this.arm = arm;
        this.hardwareArm = hardwareArm;
        this.claw = claw;
        timer = new Timer();
        timer.start();
        innerTimer = new Timer();
        innerTimer.start();
    }
    public void initialize() {
        timer.reset();
        arm.armcontrol = false;
        claw.open();
        hardwareArm.setBottomWinch(-0.1);
        hardwareArm.setTopWinch(-0.37);
        claw.TurnIn();
    }

    public void execute(){
    }

    public void end(boolean interrupted){
        System.out.println("Stopped in Startup_Intake");
        arm.zeroLength(arm.getAngle());
        arm.stop();
        arm.setLength(0);
        arm.armcontrol = true;
        claw.StopMotor();
    }

    public boolean isFinished() {
        if(timer.get()>0.5){
            if(Math.abs(hardwareArm.getBottomVelocity())<1 && Math.abs(hardwareArm.getTopVelocity())<1){
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
