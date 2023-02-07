package frc.team5115.Commands.Auto.Adjust;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Classes.Software.IntakeMotor;
import frc.team5115.Commands.Auto.FollowTrajectory;

public class AdjustDriveCommandGroup extends SequentialCommandGroup {
        Drivetrain drivetrain;
        IntakeMotor intake;

    

    public AdjustDriveCommandGroup(Drivetrain drivetrain, IntakeMotor intake){
        this.drivetrain = drivetrain;
        this.intake = intake;
        addCommands(
          new FollowTrajectory(drivetrain)
            ); 
}
}
