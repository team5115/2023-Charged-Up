package frc.team5115.Commands.Intake.CombinedIntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeExtend;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;

public class ShelfSubstation extends SequentialCommandGroup {
    Arm intake;

    public ShelfSubstation(Arm arm){
        this.intake = arm;
        addCommands(
            new IntakeExtend(arm, 0),
            new IntakeTurn(arm, 8),
            new IntakeExtend(arm, 0)
        );
    }
}
