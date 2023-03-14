package frc.team5115.Commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeExtend;

public class RealExtend extends SequentialCommandGroup {
    Arm arm;

        public RealExtend(Arm arm, double length){
            this.arm = arm;
    
            if(length > 5){
                addCommands(new IntakeExtend(arm, arm.getTopWinchLength()-1, arm.getBottomWinchLength()+2.5));
                addCommands(new IntakeExtend(arm, length-0.5, length+0.5));
    
            } else {
                addCommands(new IntakeExtend(arm, arm.getTopWinchLength()-2, arm.getBottomWinchLength() -0.5));
                addCommands(new IntakeExtend(arm, length-0.5, length-0.5));
            }
    
        }
    
}
