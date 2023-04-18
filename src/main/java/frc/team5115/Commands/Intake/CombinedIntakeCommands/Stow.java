package frc.team5115.Commands.Intake.CombinedIntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Hardware.HardwareArm;
import frc.team5115.Classes.Hardware.HardwareIntake;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.Startup_Intake;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeExtend;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;

public class Stow extends SequentialCommandGroup {
    public Stow(Arm intake, HardwareArm h, HardwareIntake I){
        addCommands(
            new IntakeExtend(intake, 0),
            new IntakeTurn(intake, -90)
            , new Startup_Intake(intake, h, I)
        );
    }
}
