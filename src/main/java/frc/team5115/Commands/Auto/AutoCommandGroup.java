package frc.team5115.Commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.*;
import frc.team5115.Classes.Hardware.*;
import frc.team5115.Commands.Auto.BasicAuto.AdjustDriveCommandGroup;
import frc.team5115.Commands.Auto.DockAuto.DockCommandGroup;
import frc.team5115.Commands.Intake.RealExtend;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.*;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;

public class AutoCommandGroup extends SequentialCommandGroup {
    Drivetrain drivetrain;
    Arm arm;
    HardwareArm hArm;
    HardwareIntake hIntake;

    public AutoCommandGroup(Drivetrain drivetrain, Arm arm, HardwareArm hArm, HardwareIntake hIntake, boolean inIdealPosition){
        this.arm = arm;
        this.drivetrain = drivetrain;
        this.hArm = hArm;
        this.hIntake = hIntake;

        setupCubeDrop();        
         if (inIdealPosition) {
            setupScuffed();
        } else {
            setupNotIdeal();
        }
        
    }
    
    private void setupCubeDrop() {
        addCommands(    
            new DriveForward(drivetrain, -0.26, 0.5), // back up to node
            new DriveForward(drivetrain, +0.65, 1.2), // speed away to drop cube
            new DriveForward(drivetrain, -0.85, 0.8) // back up to push cube into place
        );
        // this should finish with the robot pushed up against the node
    }

    private void setupIdeal() {
        addCommands(
            new HighNode(arm),
            new IntakeTurn(arm, 15),
            new Stow(arm, hArm, hIntake),
            new DriveTurn(drivetrain, 180),
            new IntakeAndMoveGroup(arm, drivetrain, 3.4, 1),
            new DriveTurn(drivetrain, 0),
            new DockCommandGroup(drivetrain, false) // dock backwards
           // ,new InstantCommand(drivetrain :: stop)
        );
    }

    private void setupNotIdeal() {
        addCommands(
            new DriveForward(drivetrain, +3, 1.0), // exit community
            new InstantCommand(drivetrain :: stop)
        );
    }

    private void setupScuffed(){
        addCommands(
        new DockCommandGroup(drivetrain, false) // dock forwards
           // ,new InstantCommand(drivetrain :: stop)
        );
    }
}
