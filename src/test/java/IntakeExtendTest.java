import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import frc.team5115.Commands.Intake.RawIntakeCommands.IntakeExtend;

public class IntakeExtendTest {

    @Test
    void testCalculate_next_step_length_large_diff() {
        final double step = IntakeExtend.calculate_next_step_length(0, +2, 20);
        Assertions.assertEquals(2, step);
    }

    @Test
    void testCalculate_next_step_length_small_diff() {

        final double step = IntakeExtend.calculate_next_step_length(10, +2, 11);
        Assertions.assertEquals(11, step);
    }

    @Test
    void testCalculate_next_step_length_step_negative_goal_negative() {
        final double step = IntakeExtend.calculate_next_step_length(10, -2, 5);
        Assertions.assertEquals(8, step);
    }

    @Test
    void testCalculate_next_step_length_step_negative_goal_positive() {
        final double step = IntakeExtend.calculate_next_step_length(10, -2, 15);
        Assertions.assertEquals(12, step);
    }

    @Test
    void testCalculate_next_step_length_step_positive_goal_negative() {
        final double step = IntakeExtend.calculate_next_step_length(10, +2, 5);
        Assertions.assertEquals(8, step);
    }

    @Test
    void testCalculate_next_step_length_in_the_negative() {
        final double step = IntakeExtend.calculate_next_step_length(-1, +2, -9);
        Assertions.assertEquals(-3, step);
    }

    @Test
    void testCalculate_next_step_length_moving_positive_across_zero() {
        final double step = IntakeExtend.calculate_next_step_length(-1, +2, 5);
        Assertions.assertEquals(+1, step);
    }
}
