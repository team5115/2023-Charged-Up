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
            // new TestingPrint("done: back up a little"),
            // new FollowTrajectory(drivetrain, +3.0, 1.2, 1),
            // new TestingPrint("done: fast forward"),
            // new FollowTrajectory(drivetrain, +3.0, 0.6, 1),
            // new TestingPrint("done: over ramp and exit community"),
            // new DockCommandGroup(drivetrain, true),
            // new TestingPrint("done: docking \n ----Auto Complete----")

        );
    }

    private void setupNotIdeal() {
        addCommands(
            new FollowTrajectory(drivetrain, -1.0, 0.5, 0.1),
            new TestingPrint("done: back up a little"),
            new FollowTrajectory(drivetrain, +1.0, 1.2, 0.1),
            new TestingPrint("done: fast forward"),
            new FollowTrajectory(drivetrain, +5.0, 1, 0.1),
            new TestingPrint("done: exit community")
        );
    }
}
