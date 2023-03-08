package frc.team5115.Commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeExtend;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;

public class RealExtend extends SequentialCommandGroup {
    Arm arm;

    public RealExtend(Arm arm, double length){
        this.arm = arm;

        if(arm.getTopWinchLength() < length || arm.getBottomWinchLength() < length){

        if(arm.getTopWinchLength() > arm.getBottomWinchLength()){
        for(double i = arm.getTopWinchLength(); i<length; i++){
            addCommands(new IntakeExtend(arm, i, i));
        }
        }
        else{
            for(double i = arm.getBottomWinchLength(); i<length; i++){
                addCommands(new IntakeExtend(arm, i, i));
            }
        }
        //addCommands(new IntakeExtend(arm, length, length));
    }
    else{
            for(double i = arm.getBottomWinchLength(); i>length; i--){
                addCommands(new IntakeExtend(arm, i, i));
            }
       // addCommands(new IntakeExtend(arm, length, length));   
    }
        /* 
        if(length > 5){
            addCommands(new IntakeExtend(arm, arm.getTopWinchLength()-2, arm.getBottomWinchLength()+3));
            addCommands(new IntakeExtend(arm, length, length+1));

        } else {
            addCommands(new IntakeExtend(arm, arm.getTopWinchLength()-2.5, arm.getBottomWinchLength() -0.5));
            addCommands(new IntakeExtend(arm, length, length-1));
        }
        */
    }
}
