package frc.team5115.Commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team5115.Classes.Hardware.HardwareArm;
import frc.team5115.Classes.Hardware.HardwareIntake;
import frc.team5115.Classes.Software.Arm;

public class Startup extends ParallelCommandGroup {
    Arm arm;
    HardwareArm hardwareArm;
    HardwareIntake hardwareIntake;

    public Startup(Arm arm, HardwareArm hardwareArm, HardwareIntake hardwareIntake){
        this.arm = arm;
        this.hardwareArm = hardwareArm;
        this.hardwareIntake = hardwareIntake;
        addCommands(new Startup_Intake(arm, hardwareArm, hardwareIntake));
        addCommands(new Startup_Angle(arm, hardwareArm));
    }

}
