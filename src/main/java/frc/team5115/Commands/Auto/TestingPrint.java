package frc.team5115.Commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestingPrint extends CommandBase{
    public TestingPrint(String string) {
        System.out.println(string);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}