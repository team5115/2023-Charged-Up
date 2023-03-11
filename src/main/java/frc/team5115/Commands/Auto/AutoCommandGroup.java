package frc.team5115.Commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Auto.BasicAuto.AdjustDriveCommandGroup;
import frc.team5115.Commands.Auto.DockAuto.DockCommandGroup;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.*;

public class AutoCommandGroup extends ParallelCommandGroup {
    Drivetrain drivetrain;
    Arm intake;

    public AutoCommandGroup(Drivetrain drivetrain, Arm intake, boolean inIdealPosition){
        this.intake = intake;
        this.drivetrain = drivetrain;

        if (inIdealPosition) {
            setupIdeal();
        } else {
            setupNotIdeal();
        }
    }

    private void setupIdeal() {
        addCommands(
            new DriveForward(drivetrain, -0.5, 0.5),
            new DriveForward(drivetrain, +1.6, 1.2),
            new DriveForward(drivetrain, -0.8, 0.8),
            new DockCommandGroup(drivetrain)
        );
    }

    private void setupNotIdeal() {
        addCommands(
            new DriveForward(drivetrain, -0.5, 0.5),
            new DriveForward(drivetrain, +1.6, 1.2)
        );
    }
}
