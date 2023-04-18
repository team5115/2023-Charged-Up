package frc.team5115.Commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team5115.Classes.Hardware.HardwareIntake;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Commands.Auto.VisionAuto.DriveForwardWVision;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.GroundPickup;

public class IntakeAndMoveGroup extends ParallelCommandGroup {
    Arm arm;

    public IntakeAndMoveGroup(Arm arm, Drivetrain d, double dist, double speed, HardwareIntake intake){
        this.arm = arm;

        addCommands(
            new GroundPickup(arm),
            new DriveForwardWVision(d, dist, speed),
            new InstantCommand(intake :: TurnIn)
        );
    }

}
