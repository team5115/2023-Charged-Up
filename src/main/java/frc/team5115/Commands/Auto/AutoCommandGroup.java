package frc.team5115.Commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5115.Classes.Software.*;
import frc.team5115.Classes.Hardware.*;
import frc.team5115.Commands.Auto.BasicAuto.AdjustDriveCommandGroup;
import frc.team5115.Commands.Auto.DockAuto.DockCommandGroup;
import frc.team5115.Commands.Intake.*;
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
        System.out.println(inIdealPosition);

        if (inIdealPosition) {
            cubeDrop();
            dockForward();
        } else {
            ehhh();
        }
    }
    
    private void cubeDrop() {
        addCommands(    
            new DriveForward(drivetrain, -0.26, 0.5), // back up to node
            new DriveForward(drivetrain, +0.65, 1.2), // speed away to drop cube
            new DriveForward(drivetrain, -0.85, 0.8) // back up to push cube into place
        );
        // this should finish with the robot pushed up against the node
    }

    private void superIdeal() {
        addCommands(
            new Startup(arm, hArm, hIntake),     
            new InstantCommand(hIntake :: TurnIn),
            new HighNode(arm),
            new IntakeTurn(arm, 10),
            new Stow(arm, hArm, hIntake),
            new DriveForward(drivetrain, -0.26, 0.5), // back up to node
            //new Stow(arm, hArm, hIntake),
            new DriveTurn(drivetrain, 180),
            new DriveForward(drivetrain, 2.4, 1), // back up to node
            //new IntakeAndMoveGroup(arm, drivetrain, 2.4, 1, hIntake),
            //new Stow(arm, hArm, hIntake),
            new DriveTurn(drivetrain, 360)
            //,new DockCommandGroup(drivetrain, false) // dock backwards
           // ,new InstantCommand(drivetrain :: stop)
        );
    }

    private void ehhh() {
        addCommands(
            new Startup(arm, hArm, hIntake),     
            new InstantCommand(hIntake :: TurnIn),
            new HighNode(arm),
            new IntakeTurn(arm, 10),
            new Stow(arm, hArm, hIntake),
            new DriveForward(drivetrain, -3.5, 1.3) // back up to node
            //new Stow(arm, hArm, hIntake),
            //new DriveTurn(drivetrain, 170),
            //new DriveForward(drivetrain, 2.4, 1), // back up to node
            //new IntakeAndMoveGroup(arm, drivetrain, 2.4, 1, hIntake),
            //new Stow(arm, hArm, hIntake),
           // new DriveTurn(drivetrain, -170)
            //,new DockCommandGroup(drivetrain, false) // dock backwards
           // ,new InstantCommand(drivetrain :: stop)
        );
    }


    private void scoreHigh() {
        // should start facing towards grid
        addCommands(
            new Startup(arm, hArm, hIntake)
             
            ,new InstantCommand(hIntake :: TurnIn),
            new WaitCommand(1),
            new StowCone(arm, hArm, hIntake),
            new InstantCommand(hIntake :: StopMotor),
            new HighNode(arm),
            new InstantCommand(hIntake :: TurnOut),
            // do something here with scoring it, like bringing the arm down
            new WaitCommand(1),
            new InstantCommand(hIntake :: StopMotor),
            new Stow(arm, hArm, hIntake)
            // ,new DriveForward(drivetrain, -3.0, 1.0)
        );
    }

    private void exitCommunity() {
        addCommands(
            new DriveForward(drivetrain, +3, 1.0), // exit community
            new InstantCommand(drivetrain :: stop)
        );
    }

    private void dockForward(){
        addCommands(
            new DockCommandGroup(drivetrain, false) // dock forwards
        );
    }
}
