package frc.team5115.Commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Drivetrain;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Auto.BasicAuto.AdjustDriveCommandGroup;
import frc.team5115.Commands.Auto.DockAuto.DockCommandGroup;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.*;

public class AutoCommandGroup extends SequentialCommandGroup {
    Drivetrain drivetrain;
    Arm intake;

    public AutoCommandGroup(Drivetrain drivetrain, Arm intake, boolean inIdealPosition){
        this.intake = intake;
        this.drivetrain = drivetrain;

        setupCubeDrop();        
         if (true) {
            setupScuffed();
        } else {
            setupNotIdeal();
        }
        
    }
    
    private void setupCubeDrop() {
        addCommands(    
            new DriveForward(drivetrain, -0.2, 0.5), // back up to node
            new DriveForward(drivetrain, +0.65, 1.2), // speed away to drop cube
            new DriveForward(drivetrain, -0.75, 1.0) // back up to push cube into place
        );
        // this should finish with the robot pushed up against the node
    }

    private void setupIdeal() {
        addCommands(
            new DriveForward(drivetrain, +3.4, 0.8), // exit community
            new DriveForward(drivetrain, -2.0, 0.6), // go over ramp and exit community
            new DockCommandGroup(drivetrain, true) // dock backwards
           // ,new InstantCommand(drivetrain :: stop)
        );
    }

    private void setupNotIdeal() {
        System.out.println("AHHHHH!!! WHY AM I RUNNING!!! that's not ideal...");
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
