package frc.team5115.Classes.Acessory;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;

public class JoyAxisBoolSupplier implements BooleanSupplier{

    private Joystick joystick;
    private int axis;
    private double threshold;
    private boolean triggerOnGreater;

    /**
     * Construct a BooleanSupplier for making Triggers that watches a joystick axis and returns true if the axis reading rises above/below a threshold
     * @param joystick the joystick to watch
     * @param axis the analog joystick on the axis
     * @param threshold the threshold value to rise above or below
     * @param triggerOnGreater true will make getAsBoolean() return true when the read value is greater than the threshold, false will do the opposite
     */
    public JoyAxisBoolSupplier(Joystick joystick, int axis, double threshold, boolean triggerOnGreater) {
        this.joystick = joystick;
        this.axis = axis;
        this.threshold = threshold;
        this.triggerOnGreater = triggerOnGreater;
    }

    /**
     * Construct a BooleanSupplier for making Triggers that watches a joystick axis and returns true if the axis reading rises above a threshold
     * @param joystick the joystick to watch
     * @param axis the analog joystick on the axis
     * @param threshold the threshold value to rise above or below
     */
    public JoyAxisBoolSupplier(Joystick joystick, int axis, double threshold) {
        this.joystick = joystick;
        this.axis = axis;
        this.threshold = threshold;
        triggerOnGreater = true;
    }

    public boolean getAsBoolean() {
        if (triggerOnGreater) {
            return joystick.getRawAxis(axis) > threshold;
        } else {
            return joystick.getRawAxis(axis) < threshold;
        }
    }
}
