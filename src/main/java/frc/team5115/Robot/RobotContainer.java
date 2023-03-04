package frc.team5115.Robot;

import frc.team5115.Classes.Software.PhotonVision;

public class RobotContainer {
    private final PhotonVision photonVision;

    public RobotContainer() {

        photonVision = new PhotonVision();
    }

    public void teleopPeriodic(){
        photonVision.Update();
    }
}
