package frc.team5115.Commands.Intake.CombinedIntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Hardware.HardwareIntake;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.StartupWinch;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeExtend;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;

public class Stow extends SequentialCommandGroup {
    public Stow(Arm arm, HardwareIntake intake){
        addCommands(
            new IntakeExtend(arm, 0),
            new IntakeTurn(arm, -90),
            new StartupWinch(arm, intake)
        );
    }
}
