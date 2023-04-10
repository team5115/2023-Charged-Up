package frc.team5115.Commands.Intake.CombinedIntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.RealExtend;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;

public class MiddleNode extends CommandBase {
    Arm intake;
    WrappedMiddleNode wrappedMiddleNode;

    public MiddleNode(Arm intake){
        this.intake = intake;
    }

    @Override
    public void initialize() {
        wrappedMiddleNode = new WrappedMiddleNode(intake);
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
                new RealExtend(intake, 0),
                new IntakeTurn(intake, angle),
                new RealExtend(intake, length)
            );
        }
    }
}
