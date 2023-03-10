package frc.team5115.Commands.Intake.CombinedIntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeExtend;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;

public class HighCone extends SequentialCommandGroup {
  Arm intake;

  public HighCone(Arm intake){
      this.intake = intake;
      addCommands(
        new IntakeExtend(intake, 0, 0),
        new IntakeTurn(intake, 20),
        new IntakeExtend(intake, 22, 22)
      );
    }

}
