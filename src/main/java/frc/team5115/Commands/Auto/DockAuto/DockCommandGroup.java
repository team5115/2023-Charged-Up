package frc.team5115.Commands.Auto.DockAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Commands.Auto.Dock;

public class DockCommandGroup extends SequentialCommandGroup {
    Drivetrain drivetrain;

    public DockCommandGroup(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addCommands(
            new DriveUntilDock(drivetrain, 8),
            new Dock(drivetrain)
        );
    }
}
