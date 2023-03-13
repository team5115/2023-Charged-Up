package frc.team5115.Commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Classes.Hardware.*;

public class Startup_Angle extends CommandBase{
    private Arm intake;
    private HardwareArm hardwareIntake;
    private HardwareIntake claw;
    private Timer timer;
    private Timer innerTimer;

    public Startup_Angle(Arm a, HardwareArm b, HardwareIntake claw){
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
        claw.open();
        hardwareIntake.setTurn(-0.1);
    }

    public void execute(){
    }

    public void end(boolean interrupted){
        System.out.println("Stopped");
        intake.zeroArm();
        intake.stop();
        intake.setLength(0);
        intake.armcontrol = true;
    }

    public boolean isFinished() {
        if(timer.get()>0.5){
            if(Math.abs(hardwareIntake.getTurnVelocity())<1){
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
