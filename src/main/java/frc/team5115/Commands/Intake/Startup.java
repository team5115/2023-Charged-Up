package frc.team5115.Commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Hardware.HardwareArm;
import frc.team5115.Classes.Hardware.HardwareIntake;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeExtend;

public class Startup extends ParallelCommandGroup {
    Arm arm;
    HardwareArm intake;
    HardwareIntake a;

    public Startup(Arm arm, HardwareArm intake, HardwareIntake a){
        this.arm = arm;
        this.intake = intake;
        this.a = a;
        addCommands(new Startup_Intake(arm, intake, a));
        addCommands(new Startup_Angle(arm, intake));
    }

}
