package frc.team5115.Classes.Software;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Paths extends SubsystemBase {
    public final PathPlannerTrajectory SideAutoPt1 = PathPlanner.loadPath("Side Auto Pt. 1", null);
	public final PathPlannerTrajectory SideAutoPt2 = PathPlanner.loadPath("Side Auto Pt. 2", null);
	public final PathPlannerTrajectory SideAutoPt3 = PathPlanner.loadPath("Side Auto Pt. 3", null);
	public final PathPlannerTrajectory SideAuto = PathPlanner.loadPath("Side Auto", null);
	public final PathPlannerTrajectory ScoreHighWithDock = PathPlanner.loadPath("Score High With Dock", null);
}
