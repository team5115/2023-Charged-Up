package frc.team5115.Commands.Intake.CombinedIntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeExtend;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;

public class MiddleNode extends CommandBase {
    Arm arm;
    WrappedMiddleNode wrappedMiddleNode;

    public MiddleNode(Arm arm){
        this.arm = arm;
    }

    @Override
    public void initialize() {
        wrappedMiddleNode = new WrappedMiddleNode(arm);
        wrappedMiddleNode.schedule();
    }

    @Override
    public final boolean isFinished() {
        return !wrappedMiddleNode.isScheduled();
    }

    private class WrappedMiddleNode extends SequentialCommandGroup {
        public WrappedMiddleNode(Arm arm) {
            boolean cubeMode = !arm.h.open;

            double length = 8;
            double angle = 10;

            if (cubeMode) {
                angle -= 1;
            }

            addCommands(
                new IntakeExtend(arm, 0),
                new IntakeTurn(arm, angle),
                new IntakeExtend(arm, length)
            );
        }
    }
}
