package frc.team5115.Commands.Intake.CombinedIntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Hardware.HardwareArm;
import frc.team5115.Classes.Hardware.HardwareIntake;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.RealExtend;
import frc.team5115.Commands.Intake.Startup;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeExtend_v2;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;

public class Stow extends SequentialCommandGroup {
    Arm intake;

    public Stow(Arm intake, HardwareArm h, HardwareIntake I){
        this.intake = intake;
        addCommands(
            new RealExtend(intake, 0),
            new IntakeTurn(intake, -90)
            , new Startup(intake, h, I)
        );
    }
}
