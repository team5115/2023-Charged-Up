package frc.team5115.Robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team5115.Classes.Software.PhotonVision;

public class Robot extends TimedRobot {
    private PhotonVision photonVision;

    @Override
    public void robotInit() {
        photonVision = new PhotonVision();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopPeriodic() {
        photonVision.Update();
    }
}
