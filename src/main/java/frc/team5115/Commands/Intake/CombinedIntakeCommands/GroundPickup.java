package frc.team5115.Commands.Intake.CombinedIntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeExtend;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;

public class GroundPickup extends SequentialCommandGroup {
    public GroundPickup(Arm arm){
        addCommands(
            new IntakeExtend(arm, 0),
            new IntakeTurn(arm, -45),
            new IntakeExtend(arm, 17.5)
        );
    }
}
