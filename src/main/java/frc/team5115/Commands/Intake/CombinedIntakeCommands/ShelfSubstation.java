package frc.team5115.Commands.Intake.CombinedIntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.RealExtend;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeExtend_v2;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;

public class ShelfSubstation extends SequentialCommandGroup {
    Arm intake;

    public ShelfSubstation(Arm intake){
        this.intake = intake;
        addCommands(
            new RealExtend(intake, 0),
            new IntakeTurn(intake, 15.5),
            new RealExtend(intake, 0)
        );
    }
}
