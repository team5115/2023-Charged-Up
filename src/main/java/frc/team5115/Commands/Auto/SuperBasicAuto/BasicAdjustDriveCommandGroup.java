package frc.team5115.Commands.Auto.SuperBasicAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Commands.Auto.*;

public class BasicAdjustDriveCommandGroup extends SequentialCommandGroup {
        Drivetrain drivetrain;

    public BasicAdjustDriveCommandGroup(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addCommands(
            new DriveForward(drivetrain, -1),
            new DriveForward(drivetrain, 4)
        ); 
}
}
