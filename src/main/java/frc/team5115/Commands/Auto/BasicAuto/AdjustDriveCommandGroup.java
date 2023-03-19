package frc.team5115.Commands.Auto.BasicAuto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Commands.Auto.FollowTrajectory;

public class AdjustDriveCommandGroup extends SequentialCommandGroup {
    Drivetrain drivetrain;
    Arm intake;
    ArrayList<Translation2d> interiorWaypoints;

    public AdjustDriveCommandGroup(Drivetrain drivetrain, Arm intake){
        this.drivetrain = drivetrain;
        this.intake = intake;

        interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(-1, 0));

        addCommands(
            new FollowTrajectory(drivetrain, -4.0, -1.0, 0.0, interiorWaypoints)
        ); 
}
}
