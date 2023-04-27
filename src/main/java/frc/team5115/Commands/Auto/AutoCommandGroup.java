package frc.team5115.Commands.Auto;

import java.util.HashMap;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.*;
import frc.team5115.Classes.Hardware.*;
import frc.team5115.Commands.Auto.DockAuto.DockCommandGroup;
import frc.team5115.Commands.Intake.*;
import frc.team5115.Commands.Intake.CombinedIntakeCommands.*;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;
import frc.team5115.Classes.Software.Paths;

public class AutoCommandGroup extends SequentialCommandGroup {
    Drivetrain drivetrain;
    Arm arm;
    HardwareArm hArm;
    HardwareIntake hIntake;
	Paths paths;

    public AutoCommandGroup(Drivetrain drivetrain, Arm arm, HardwareArm hArm, HardwareIntake hIntake, boolean inIdealPosition){
        this.arm = arm;
        this.drivetrain = drivetrain;
        this.hArm = hArm;
        this.hIntake = hIntake;
		this.paths = new Paths();
/* 
        addCommands(
            new Stow(arm, hArm, hIntake),
            new DriveForwardWVision(drivetrain, -0.5, 0.3),
            new DriveTurn(drivetrain, 180)
            , new WaitCommand(1)
            ,new IntakeAndMoveGroup(arm, drivetrain, 0.8, 0.3, hIntake),
            new StowCone(arm, hArm, hIntake),
            new DriveTurn(drivetrain, 0)

        );
        */
// /* 
        if (inIdealPosition) {
            //cubeDrop();
            scoreHighWDock();
            dockForward();
        } else {
			BasicHighNode();
        }
// */
        }
    
	/**
	 * Places a cube in a low node. Expects that the robot has its back facing the grid and has a cube on its base.
	 */
    private void cubeDrop() {
        addCommands(    
            new DriveForward(drivetrain, -0.26, 0.5), // back up to node
            new DriveForward(drivetrain, +0.65, 1.2), // speed away to drop cube
            new DriveForward(drivetrain, -0.85, 0.8) // back up to push cube into place
        );
        // this should finish with the robot pushed up against the node
    }

	/**
	 * Scores two cones in high nodes, one that starts in the intake and one that is picked up from the ground. Ends outside of the community.
	 */
    private void PathPlannerHighNodeOld() {
		addCommands(
            new Startup(arm, hArm, hIntake),
            new InstantCommand(hIntake :: TurnIn),
            new HighNode(arm),
            new IntakeTurn(arm, 10),
			new InstantCommand(hIntake :: StopMotor),
            new Stow(arm, hArm, hIntake),
			drivetrain.getRamseteCommand(paths.SideAutoPt1),
			new GroundPickup(arm),
			new InstantCommand(hIntake :: TurnIn),
			drivetrain.getRamseteCommand(paths.SideAutoPt2),
			new HighNode(arm),
			new IntakeTurn(arm, 10),
			new InstantCommand(hIntake :: StopMotor),
			new StowCone(arm),
			drivetrain.getRamseteCommand(paths.SideAutoPt3)
		);
    }

	/**
	 * Scores two cones in high nodes, one that starts in the intake and one that is picked up from the ground. Ends outside of the community. Uses PathPlanner instead of `DriveForward`. The path uses Stop Events to run commands as part of the trajectory following.
	 */
	private void PathPlannerHighNode() {
		HashMap<String, Command> eventMap = new HashMap<>();
		eventMap.put("Stow", new Stow(arm, hArm, hIntake));
		eventMap.put("Intake", new InstantCommand(hIntake :: TurnIn));
		eventMap.put("High Node", new HighNode(arm));
		eventMap.put("Arm Down 10", new IntakeTurn(arm, 10));
		eventMap.put("Stop Intake", new InstantCommand(hIntake :: StopMotor));
		eventMap.put("Stow With Cone", new StowCone(arm));
		eventMap.put("Ground Pickup", new GroundPickup(arm));
		eventMap.put("Down & Stow", new IntakeTurn(arm, 10).andThen(new Stow(arm, hArm, hIntake)));

		addCommands(new FollowPathWithEvents(drivetrain.getRamseteCommand(paths.SideAuto), paths.SideAuto.getMarkers(), eventMap));
	}

	/**
	 * Docks the robot by moving forward.
	 */
    private void dockForward(){
        addCommands(
            new DockCommandGroup(drivetrain, false), // dock forwards
            new InstantCommand(drivetrain ::  stop)
        );
    }

	/**
	 * Exits the community.
	 */
    private void exitCommunity() {
        addCommands(
            new DriveForward(drivetrain, +3, 1.0), // exit community
            new InstantCommand(drivetrain :: stop)
        );
    }

	/**
	 * Exits the community. Uses PathPlanner instead of the `DriveForward`.
	 */
	private void exitCommunityPathPlanner() {
		addCommands(
			drivetrain.getRamseteCommand(paths.ExitCommunity)
		);
	}

	/**
	 * Scores a cone in a high node.
	 */
    private void BasicHighNode() {
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

	/**
	 * Scores a cone in a high node, then moves the robot into a good position to begin docking with `dockForward`.
	 */
    private void scoreHighWDock() {
        // should start facing towards grid
        addCommands(
            new Startup(arm, hArm, hIntake),     
            new InstantCommand(hIntake :: TurnIn),
            new HighNode(arm),
            new IntakeTurn(arm, 10),
            new StowCone(arm),
            new InstantCommand(hIntake :: StopMotor),
            new DriveForward(drivetrain, -0.25, 1),
            new DriveTurn(drivetrain, 180)
        );
    }

	/**
	 * Scores a cone in a high node, then moves the robot into a good position to begin docking with `dockForward`. Uses PathPlanner instead of `DriveForward`.
	 */
	private void scoreHighWDockPathPlanner() {
		addCommands(
			new Startup(arm, hArm, hIntake),
			new InstantCommand(hIntake :: TurnIn),
			new HighNode(arm),
			new IntakeTurn(arm, 10),
			new InstantCommand(hIntake :: StopMotor),
			new Stow(arm, hArm, hIntake),
			drivetrain.getRamseteCommand(paths.ScoreHighWithDock)
		);
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

}
