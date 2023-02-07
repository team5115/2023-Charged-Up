package frc.team5115.Commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Classes.Software.IntakeMotor;
import frc.team5115.Commands.Auto.Adjust.AdjustDriveCommandGroup;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.*;

public class AutoCommandGroup extends ParallelCommandGroup {
  Drivetrain drivetrain;
  IntakeMotor intake;

  public AutoCommandGroup(Drivetrain drivetrain, IntakeMotor intake){
      this.intake = intake;
      this.drivetrain = drivetrain;
      addCommands(
          //shoot preloaded ball
          new AdjustDriveCommandGroup(drivetrain, intake),
          new HighCone(intake)
          );
          //Substitute for limelight code
          //new DriveToPoint(drivetrain),
    }

}
