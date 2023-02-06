package frc.team5115.Commands.Auto.Adjust;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Drivetrain;

public class AdjustAngle extends CommandBase {
    Drivetrain drivetrain;

      

    public AdjustAngle(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
        public void initialize(){
        }

    @Override
        public void execute() {
            drivetrain.AdjustAngle();
            System.out.print("adjust angle x: " + drivetrain.getX());
        }

    @Override 
        public void end(boolean interputed){
            System.out.println("adjust angle is finished");
        }

    @Override
        public boolean isFinished() {
            if(Math.abs(drivetrain.getX()) < 0.5){
                return true;
            }
            else{
                return false;
            }
        }
}