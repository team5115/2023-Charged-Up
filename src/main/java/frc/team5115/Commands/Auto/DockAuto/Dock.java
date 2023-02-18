package frc.team5115.Commands.Auto.DockAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Drivetrain;

public class Dock extends CommandBase{
    private Timer dockedTimer;
    private Drivetrain drivetrain;
    private double encoderDistanceAtStart;
    private final PIDController dockPID;
    public static final double dockP = 0.0175;
    private static final double MaxTravelDistance = 1.37;
    public static final double BalancingTolerance = 1.5; // tolerance in degrees from 0 to decide if the robot is balanced/docked

    public Dock(Drivetrain drivetrain) {
        dockPID = new PIDController(dockP, 0, dockP/10);
        this.drivetrain = drivetrain;
        dockedTimer = new Timer();
    }

    @Override
    public void initialize() {
        encoderDistanceAtStart = drivetrain.getLeftDistance();
    }

    @Override
    public void execute() {
        // System.out.println("dist from start: " + getDistanceFromStart());
        double pitch = drivetrain.getPitchDeg();
        // PID loop tries to go towards the setpoint, so in general, a positive currentValue and a 0 setpoint will return negative output
        // this is why it actually runs at the opposite of what the PID loop says
        double forward = -dockPID.calculate(pitch, 0);
        // System.out.println("Docking @ " + forward + " m/s");

        if (getDistanceFromStart() < MaxTravelDistance) {
            drivetrain.autoDrive(forward);
        } else {
            // System.out.println("Stopped docking - too close to edge!");
            drivetrain.stop();
        }
        
        if (Math.abs(pitch) < BalancingTolerance) {
            dockedTimer.start();
            // System.out.println("balanced");
        } else {
            dockedTimer.reset();
        }
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.setThrottleEnabled(true);
        System.out.println("Successfully docked!");
        System.out.println("end angle: " + drivetrain.getPitchDeg() + " degrees");
        System.out.println("distance from start: " + getDistanceFromStart() + " meters");
    }

    @Override
    public boolean isFinished() {
        // finish if docked for more than the minimum dock time
        if (dockedTimer.get() > 3) {
            return true;
        }
        return false;
    }

    private double getDistanceFromStart() {
        return drivetrain.getLeftDistance() - encoderDistanceAtStart;
    }
}