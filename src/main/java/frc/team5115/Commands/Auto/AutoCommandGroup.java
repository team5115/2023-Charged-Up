package frc.team5115.Commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Commands.Auto.BasicAuto.AdjustDriveCommandGroup;

public class AutoCommandGroup extends ParallelCommandGroup {
  Drivetrain drivetrain;

  public AutoCommandGroup(Drivetrain drivetrain){
      this.drivetrain = drivetrain;
      addCommands(
          //shoot preloaded ball
          new AdjustDriveCommandGroup(drivetrain)
          );
          //Substitute for limelight code
          //new DriveToPoint(drivetrain),
    }

}
