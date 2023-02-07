package frc.team5115.Commands.Auto.Adjust;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Classes.Software.IntakeMotor;

@Deprecated
public class AdjustDistance extends CommandBase {
    Drivetrain drivetrain;
    IntakeMotor intake;
    Timer timer;

    public AdjustDistance(Drivetrain drivetrain, IntakeMotor intake) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        timer = new Timer();
        timer.start();
    }

    @Override
        public void initialize(){
            timer.reset();
        }


    @Override
        public void execute() {
            drivetrain.AdjustDistance();

        }

    @Override
        public void end(boolean interrupted){
            drivetrain.stop();
        }

    
    @Override
        public boolean isFinished() {
            if(timer.get() >  4.0){
                return true;
            }
            return false;
        }
}

