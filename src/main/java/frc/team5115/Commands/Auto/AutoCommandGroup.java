package frc.team5115.Commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Auto.BasicAuto.AdjustDriveCommandGroup;
import frc.team5115.Commands.Auto.DockAuto.DockCommandGroup;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.*;

public class AutoCommandGroup extends SequentialCommandGroup {
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
            new DriveForward(drivetrain, -1.0, 0.5),
            new TestingPrint("done first"),
            new DriveForward(drivetrain, +3.0, 1.2),
            new TestingPrint("done second"),
            new DriveForward(drivetrain, +2.0, 0.6),
            new TestingPrint("done third"),
            new DriveForward(drivetrain, -2.0, 0.8),
            new TestingPrint("done fourth"),
            new DockCommandGroup(drivetrain),
            new TestingPrint("done completely")

        );
    }

    private void setupNotIdeal() {
        addCommands(
            new DriveForward(drivetrain, +1, 0.5),
            new DriveForward(drivetrain, -1, 0.5)
        );
    }
}
