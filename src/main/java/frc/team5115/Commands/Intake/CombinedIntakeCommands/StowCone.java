package frc.team5115.Commands.Intake.CombinedIntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.RawIntakeCommands.*;

public class StowCone extends SequentialCommandGroup {
    public StowCone(Arm arm){
        addCommands(
            new IntakeExtend(arm, 0),
            new IntakeTurn(arm, -80)
        );
    }
}
