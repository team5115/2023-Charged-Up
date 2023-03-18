package frc.team5115.Commands.Auto.DockAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Drivetrain;

public class Dock extends CommandBase{
    private Timer dockedTimer;
    private Drivetrain drivetrain;
    private double encoderDistanceAtStart;
    private final double direction;
    private final PIDController dockPID;
    public final double dockP;
    private double last_pitch = 0;
    private final double MaxTravelDistance;
    public final double BalancingTolerance; // tolerance in degrees from 0 to decide if the robot is balanced/docked

    public Dock(Drivetrain drivetrain, double direction) {
        this.direction = direction;
        MaxTravelDistance = 1.5 * direction;
        BalancingTolerance = 2;
        dockP = 0.0174;
        dockPID = new PIDController(dockP, 0, dockP/5);
        this.drivetrain = drivetrain;
        dockedTimer = new Timer();
    }

    @Override
    public void initialize() {
        last_pitch = drivetrain.getPitchDeg();
        encoderDistanceAtStart = drivetrain.getLeftDistance();
    }

    @Override
    public void execute() {
        // System.out.println("dist from start: " + getDistanceFromStart());
        double pitch = drivetrain.getPitchDeg();
        // PID loop tries to go towards the setpoint, so in general, a positive currentValue and a 0 setpoint will return negative output
        // this is why it actually runs at the opposite of what the PID loop says
        // if direction is -1, then the pid loop will just be reversed and forward will be negative without needing to be multiplied by direction
        double forward = -dockPID.calculate(pitch, 0);
        // System.out.println("Docking @ " + forward + " m/s");

        if (getDistanceFromStart() * direction < MaxTravelDistance * direction && Math.abs(pitch - last_pitch) < 1) {
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
        last_pitch = pitch;
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.setThrottleEnabled(true);

        if (interrupted) {
            System.out.println("docking interrupted");
        } else {
            System.out.println("Successfully docked!");
        }

        System.out.println("end angle: " + drivetrain.getPitchDeg() + " degrees");
        System.out.println("distance from start: " + getDistanceFromStart() + " meters");
    }

    @Override
    public boolean isFinished() {
        // finish if docked for more than the minimum dock time
        if (dockedTimer.get() > 0.32) {
            return true;
        }
        return false;
    }

    private double getDistanceFromStart() {
        return drivetrain.getLeftDistance() - encoderDistanceAtStart;
    }
}