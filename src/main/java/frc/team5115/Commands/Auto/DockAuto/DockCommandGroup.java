package frc.team5115.Commands.Auto.DockAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Drivetrain;

public class DockCommandGroup extends SequentialCommandGroup {

    public DockCommandGroup(Drivetrain drivetrain, boolean goBackwards) {
        double direction = goBackwards ? -1 : 1;

        addCommands(
            new DriveUntilDock(drivetrain, direction),
            new Dock(drivetrain, direction)
        );
    }
}
