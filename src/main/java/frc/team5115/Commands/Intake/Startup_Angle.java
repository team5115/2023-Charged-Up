package frc.team5115.Commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Classes.Hardware.*;

public class Startup_Angle extends CommandBase {
    private Arm arm;
    private HardwareArm hardwareArm;
    private Timer timer;
    private Timer innerTimer;

    public Startup_Angle(Arm arm, HardwareArm hardwareArm){
        this.arm = arm;
        this.hardwareArm = hardwareArm;
        timer = new Timer();
        timer.start();
        innerTimer = new Timer();
        innerTimer.start();
    }
    public void initialize() {
        timer.reset();
        arm.armcontrolangle = false;
        hardwareArm.setTurn(-0.3);
    }

    public void execute(){
    }

    public void end(boolean interrupted){
        System.out.println("Stopped in Startup_Angle");
        arm.zeroArm();
        arm.stop();
        arm.setLength(0);
        arm.armcontrolangle = true;
    }

    public boolean isFinished() {
        if(timer.get()>0.07){
            if(Math.abs(hardwareArm.getTurnVelocity())<1.5){
            innerTimer.start();
            if(innerTimer.get() > 0.3) return true;
        }
        else{
            innerTimer.reset();
        }

        if(timer.get()> 0.2) return true;
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
