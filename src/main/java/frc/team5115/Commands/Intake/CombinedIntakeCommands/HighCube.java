package frc.team5115.Commands.Intake.CombinedIntakeCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeExtend;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;

public class HighCube extends ParallelCommandGroup {
  Arm intake;

  public HighCube(Arm intake){
      this.intake = intake;
      addCommands(
        new IntakeExtend(intake, 0, 0),
        new IntakeTurn(intake, 0),
        new IntakeExtend(intake, 0, 0)
        );
    }

}
