package frc.team5115.Commands.Intake.CombinedIntakeCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team5115.Classes.Software.IntakeMotor;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeExtend;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;

public class HighCone extends ParallelCommandGroup {
  IntakeMotor intake;

  public HighCone( IntakeMotor intake){
      this.intake = intake;
      addCommands(
        new IntakeExtend(intake, 0),
        new IntakeTurn(intake, 16),
        new IntakeExtend(intake, 22)
          );
    }

}
