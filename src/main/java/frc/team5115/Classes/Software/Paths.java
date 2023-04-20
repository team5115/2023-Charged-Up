package frc.team5115.Classes.Software;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Paths extends SubsystemBase {
    public final PathPlannerTrajectory badAutoPt1 = PathPlanner.loadPath("Bad Auto Pt. 1", null);
	public final PathPlannerTrajectory badAutoPt2 = PathPlanner.loadPath("Bad Auto Pt. 2", null);
	public final PathPlannerTrajectory badAutoPt3 = PathPlanner.loadPath("Bad Auto Pt. 3", null);
	public final PathPlannerTrajectory SideAuto = PathPlanner.loadPath("Side Auto", null);
}
