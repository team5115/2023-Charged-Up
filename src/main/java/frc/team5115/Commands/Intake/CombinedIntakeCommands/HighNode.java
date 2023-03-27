package frc.team5115.Commands.Intake.CombinedIntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Commands.Intake.RealExtend;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeTurn;

public class HighNode extends CommandBase {
    Arm intake;
    WrappedHighNode wrappedHighNode; 

    public HighNode(Arm intake){
        this.intake = intake;
    }

    @Override
    public void initialize() {
        wrappedHighNode = new WrappedHighNode(intake);
        wrappedHighNode.schedule();
    }

    @Override
    public final boolean isFinished() {
        return wrappedHighNode.isFinished();
    }

    private class WrappedHighNode extends SequentialCommandGroup {
        public WrappedHighNode(Arm arm) {
            boolean cubeMode = !arm.h.open;

            double length = 25;
            double angle = 20;

            if (cubeMode) {
                length -= 7;
                angle -= 2;
            }

            addCommands(
                new RealExtend(intake, 0),
                new IntakeTurn(intake, angle),
                new RealExtend(intake, length)
            );
        }
    }
}
