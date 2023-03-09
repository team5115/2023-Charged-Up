package frc.team5115.Commands.Auto.BasicAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Commands.Auto.*;
import frc.team5115.Commands.Auto.DockAuto.Dock;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;

public class ChargingAdjustDriveCommandGroup extends SequentialCommandGroup {
        Drivetrain drivetrain;
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();

    public ChargingAdjustDriveCommandGroup(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        interiorWaypoints.add(new Translation2d(-1, 0));
        interiorWaypoints.add(new Translation2d(8, 0));  
        addCommands(
          new FollowTrajectory(drivetrain, -4.0, -1.0, 0.0, interiorWaypoints),
          new Dock(drivetrain)
        ); 
}
}
