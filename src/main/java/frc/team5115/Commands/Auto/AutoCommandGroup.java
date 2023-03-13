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
            new FollowTrajectory(drivetrain, 0.1, 0.1, 0.1)
            // new DriveForward(drivetrain, -0.33, 0.5), // back up to node
            // new DriveForward(drivetrain, +0.65, 1.2), // speed away to drop cube
            // new DriveForward(drivetrain, +3.0, 0.6), // go over ramp
            // new DriveForward(drivetrain, +0.5, 0.8), // exit community
            // new DockCommandGroup(drivetrain, true) // dock backwards
        );
    }

    private void setupNotIdeal() {
        addCommands(
            new DriveForward(drivetrain, -0.33, 0.5), // back up to node
            new DriveForward(drivetrain, +0.65, 1.2), // speed away to drop cube
            new DriveForward(drivetrain, +3.5, 1.0) // go over ramp
        );
    }
}
