package frc.team5115.Commands.Auto.SuperBasicAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Auto.*;

public class BasicAdjustDriveCommandGroup extends SequentialCommandGroup {
        Drivetrain drivetrain;
        Arm intake;

    public BasicAdjustDriveCommandGroup(Drivetrain drivetrain, Arm intake){
        this.drivetrain = drivetrain;
        this.intake = intake;
        addCommands(
            new DriveForward(drivetrain, -1),
            new DriveForward(drivetrain, 4)
        ); 
}
}
