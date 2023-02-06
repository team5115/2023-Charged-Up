package frc.team5115.Commands.Auto.Adjust;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Classes.Software.IntakeMotor;

public class AdjustDriveCommandGroup extends SequentialCommandGroup {
        Drivetrain drivetrain;
        IntakeMotor intake;

    

    public AdjustDriveCommandGroup(Drivetrain drivetrain, IntakeMotor intake){
        this.drivetrain = drivetrain;
        this.intake = intake;
        addCommands(
        //Adjusts Angle
        //new AdjustAngle(drivetrain),

       new AdjustDistance(drivetrain, intake)

     //   new Stop(drivetrain)
        );
        
}
}
