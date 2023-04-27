package frc.team5115.Classes.Software;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Paths extends SubsystemBase {
    private final double MaxSpeed = 0.1; // m/s
    private final double MaxAcceleration = 0.1; // m/s^2
    public final PathConstraints constraints = new PathConstraints(MaxSpeed, MaxAcceleration);

    public final PathPlannerTrajectory SideAutoPt1;
	public final PathPlannerTrajectory SideAutoPt2;
	public final PathPlannerTrajectory SideAutoPt3;
	public final PathPlannerTrajectory SideAuto;
	public final PathPlannerTrajectory ScoreHighWithDock;
	public final PathPlannerTrajectory ExitCommunity;

    public Paths() {
        SideAutoPt1 = PathPlanner.loadPath("Side Auto Pt. 1", constraints);
        SideAutoPt2 = PathPlanner.loadPath("Side Auto Pt. 2", constraints);
        SideAutoPt3 = PathPlanner.loadPath("Side Auto Pt. 3", constraints);
        SideAuto = PathPlanner.loadPath("Side Auto", constraints);
        ScoreHighWithDock = PathPlanner.loadPath("Score High With Dock", constraints);
        ExitCommunity = PathPlanner.loadPath("Exit Community", constraints);
    }
}
