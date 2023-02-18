package frc.team5115.Classes.Acessory;

public class ThrottleControl{
    
    private boolean isThrottleSwitched;
    private boolean isSlowModeEnabled;
    private double primaryThrottle;
    private double secondaryThrottle;
    private double slowModeMultiplier;
    private boolean enabled;
 
    /**
     * Main use is to multiply getThrottle() by an input [-1, 1] to get a value that is adjusted based on throttle and slow-mode.
     * @param primaryThrottle the primary throttle to return
     * @param secondaryThrottle the other throttle to return, usually the negative version of the primary throttle
     * @param slowModeMultiplier the value to multiply the throttle by if slow mode is on
     */
    public ThrottleControl(double primaryThrottle, double secondaryThrottle, double slowModeMultiplier){
        this.primaryThrottle = primaryThrottle;
        this.secondaryThrottle = secondaryThrottle;
        this.slowModeMultiplier = slowModeMultiplier;
        isThrottleSwitched = false;
        isSlowModeEnabled = false;
        enabled = true;
    }
    
    public void toggleThrottle(){
        isThrottleSwitched = !isThrottleSwitched;
    }

    public void toggleSlowMode() {
        isSlowModeEnabled = !isSlowModeEnabled;
    }

    /**
     * @param isThrottleSwitched set to true to use secondary throttle, false to use primary throttle
     */
    public void setThrottleSwitched(boolean isThrottleSwitched) {
        this.isThrottleSwitched = isThrottleSwitched;
    }

    public void setThrottleEnabled(boolean enable) {
        enabled = enable;
    }

    public boolean getThrottleSwitched() {
        return isThrottleSwitched;
    }

    /**
     * @return the chosen throttle (primary or secondary) times the slowmode multiplier (if it's enabled)
     */
    public double getThrottle() {
        // look up ternary operator if this doesn't make sense
        if (enabled) {
            return (isThrottleSwitched ? secondaryThrottle : primaryThrottle) * (isSlowModeEnabled ? slowModeMultiplier : 1);
        } else {
            return 0;
        }
    }
 
    public String toString(){
        return "Throttle: " + getThrottle();
    }
 
}