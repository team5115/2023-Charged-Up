package frc.team5115.Commands.Intake.CombinedIntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Hardware.HardwareIntake;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.RealExtend;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeExtend_v2;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;

public class HighNode extends SequentialCommandGroup {
    Arm intake;

    public  HighNode(Arm intake){
        this.intake = intake;
        final HardwareIntake claw = intake.h;
        boolean cubeMode = !claw.open;

        double length = 25;
        double angle = 20;

        if (cubeMode) {
            length -= 7;
            angle -= 2;
        }

        addCommands(
            new RealExtend(intake, 0),
            new IntakeTurn(intake, angle),
            new RealExtend(intake, length)
            //, new IntakeTurn(intake, 15) not getting implemented until arm has consistent tensioning b/c this can cause nicking & arm lowering before needed driver input
            //you cannot say one day testing where this code works disproves this claim because the tensioning changes between days, and that is why driver input is needed
            // to correct this 
        );
    }
}
