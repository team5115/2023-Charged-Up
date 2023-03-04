package frc.team5115.Robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        CameraServer.startAutomaticCapture();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
    }


    public void autonomousPeriodic() {
    }

    public void teleopInit () {
        CameraServer.startAutomaticCapture();
    }
    
    public void teleopPeriodic () {
       robotContainer.teleopPeriodic();
    }

    public void testInit () {

    }

    public void testPeriodic () {

    }

    public void practiceInit(){
        CameraServer.startAutomaticCapture();
    }

    public void practicePeriodic(){

    }
}
