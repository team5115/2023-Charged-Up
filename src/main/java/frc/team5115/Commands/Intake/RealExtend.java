package frc.team5115.Commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeExtend;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;

public class RealExtend extends SequentialCommandGroup {
  Arm arm;

  public RealExtend(Arm arm, double length){
      this.arm = arm;
      if(arm.getTopWinchLength() - length> 0 ){
      addCommands(
        new IntakeExtend(arm, arm.getTopWinchLength() - 2, arm.getBottomWinchLength())
      );
      }
      else{
        addCommands(
            new IntakeExtend(arm, arm.getTopWinchLength(), arm.getBottomWinchLength() + 4)
          );
      }
      addCommands(
        new IntakeExtend(arm, length, length)
      );
    }

}
