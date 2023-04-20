package frc.team5115.Classes.Software;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Paths extends SubsystemBase {
	public final PathPlannerTrajectory SideAuto = PathPlanner.loadPath("Side Auto", null);
}
