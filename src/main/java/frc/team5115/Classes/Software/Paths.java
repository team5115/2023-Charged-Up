package frc.team5115.Classes.Software;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

/**
 * A collection of all the paths used by the robot.
 */
public class Paths {
    public final static PathPlannerTrajectory SideAutoPt1 = PathPlanner.loadPath("Side Auto Pt. 1", null);
	public final static PathPlannerTrajectory SideAutoPt2 = PathPlanner.loadPath("Side Auto Pt. 2", null);
	public final static PathPlannerTrajectory SideAutoPt3 = PathPlanner.loadPath("Side Auto Pt. 3", null);
	public final static PathPlannerTrajectory SideAuto = PathPlanner.loadPath("Side Auto", null);
	public final static PathPlannerTrajectory ScoreHighWithDock = PathPlanner.loadPath("Score High With Dock", null);
	public final static PathPlannerTrajectory ExitCommunity = PathPlanner.loadPath("Exit Community", null);
}
