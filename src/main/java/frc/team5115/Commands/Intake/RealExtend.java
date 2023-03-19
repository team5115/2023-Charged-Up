package frc.team5115.Commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.RawIntakeCommands.*;

public class RealExtend extends SequentialCommandGroup {
    Arm arm;

    public RealExtend(Arm arm, double length){
        this.arm = arm;

        addCommands(
        new IntakeExtend_v2(arm, length, length)
        );

        /* Old
        if(length > 5){
            addCommands(new IntakeExtend(arm, arm.getTopWinchLength()-2, arm.getBottomWinchLength()+3));
            addCommands(new IntakeExtend(arm, length, length+1));

        } else {
            addCommands(new IntakeExtend(arm, arm.getTopWinchLength()-2.5, arm.getBottomWinchLength() -0.5));
            addCommands(new IntakeExtend(arm, length-1, length-1));
            
        } */

    }

}