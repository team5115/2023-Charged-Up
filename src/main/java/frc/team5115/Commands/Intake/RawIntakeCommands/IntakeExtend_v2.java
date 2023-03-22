package frc.team5115.Commands.Intake.RawIntakeCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5115.Classes.Software.Arm;
import frc.team5115.Classes.Hardware.*;

public class IntakeExtend_v2 extends CommandBase{
    private Arm intake;
    private Timer timer;
    private Timer innerTimer;
    private double topLength_target;
    private double bottomLength_target;
    double bottomSpeed = 0;
    double topSpeed = 0;
    final double max_delta_length=Math.abs(2); // inches, should be positve
    final double suggested_length_step_top=max_delta_length;      // arbitrary fraction of max value
    final double suggested_length_step_bottom=max_delta_length; // arbitrary fraction of max value
    final double min_delta_length_when_seperated = Math.abs(2.5);
    final double max_delta_length_when_seperated = Math.abs(3.5);

    final double allowed_error_in_length_to_finish_top=1;     // how close to the final target length do you need to be
    final double allowed_error_in_length_to_finish_bottom=1; // how close to the final target length do you need to be

    boolean both_arms_are_at_target_value=false;


    public IntakeExtend_v2(Arm a, double topLength_target, double bottomLength_target){
        intake = a;
        this.topLength_target = topLength_target;
        this.bottomLength_target = bottomLength_target;
        timer = new Timer();
        timer.start();
        innerTimer = new Timer();
        innerTimer.start();
    }
    public void initialize() {
        timer.reset();
        innerTimer.reset();
        intake.setTopPID(0.45);
        intake.setBottomPID(0.5);
        // nothing will happen till the first loop execute
        //intake.topWinchSetLength(topLength_target);
        //intake.bottomWinchSetLength(bottomLength_target);
        both_arms_are_at_target_value=false;
        if(topLength_target > 20 && bottomLength_target > 20 && !intake.h.open){
            //topLength_target = 20;
            //bottomLength_target = 20;
        }
        System.out.println(intake.h.open);
    }


   private double calculate_next_step_length(double current_length, double suggested_length_step, double target_length) {
        /**
         * calculates the next length value usually based on step, but limits at target
         * @return the new length for this step
         */

        // currently this is for positive only logic
        // probably works for negatives but need to go through

        final double length_remaining = target_length - current_length;
        final double length_remaining_sign = Math.signum(length_remaining);
        final double length_remaining_magnitude = Math.abs(length_remaining);

        // probably always positive, but lets be sure
        final double suggested_length_step_magnitude = Math.abs(suggested_length_step);

        final double actual_length_step_magnitude = Math.min(suggested_length_step_magnitude, length_remaining_magnitude);
        final double actual_length_step_sign = length_remaining_sign;

        final double actual_length_step = actual_length_step_magnitude * actual_length_step_sign;

        final double new_length = current_length + actual_length_step;

        return new_length;
    }


    private void pause_top()
    {
        double current_length = intake.getTopWinchLength();
        double new_length=calculate_next_step_length(current_length,suggested_length_step_top,topLength_target);
        intake.topWinchSetLength(current_length+((new_length-current_length)/1.7));
      //  System.out.println("Disable Top");
      //  intake.slowBottomPID();
      //  intake.stepTopPID();
    }

    private void step_top(){
        double current_length = intake.getTopWinchLength();
        double new_length=calculate_next_step_length(current_length,suggested_length_step_top,topLength_target);
        intake.topWinchSetLength(new_length);
      //  System.out.println("Enable Top");
    }
    private void pause_bottom()
    {
        double current_length = intake.getBottomWinchLength();
        double new_length=calculate_next_step_length(current_length,suggested_length_step_bottom,bottomLength_target);
        intake.bottomWinchSetLength(current_length+((new_length-current_length)/1.7));
       // System.out.println("Disable Bottom");
      // intake.slowTopPID();
      // intake.stepBottomPID();
    }

    private void step_bottom(){
        double current_length = intake.getBottomWinchLength();
        double new_length=calculate_next_step_length(current_length,suggested_length_step_bottom,bottomLength_target);
        intake.bottomWinchSetLength(new_length);
        //System.out.println("Enable Bottom");
    }

    public void execute(){

        /////////////////////////////////////////////////////////////////
        //
        //  get all the numbers
        //
        /////////////////////////////////////////////////////////////////

        // measure the current lengths
        final double current_length_bottom=intake.getBottomWinchLength();
        final double current_length_top=intake.getTopWinchLength();

        // calcuate all the lengths
        final double delta_length=current_length_top-current_length_bottom;

        // calculate the remaining length for top and bottom
        final double remaining_length_top=topLength_target-current_length_top;
        final double remaining_length_top_magnitude=Math.abs(remaining_length_top);
        //final double remaining_length_top_sign=Math.signum(remaining_length_top);

        final double remaining_length_bottom=bottomLength_target-current_length_bottom;
        final double remaining_length_bottom_magnitude=Math.abs(remaining_length_bottom);

        // Overhung when top is further out
        final boolean is_overhung = ((current_length_top + min_delta_length_when_seperated) > current_length_bottom);

        final boolean is_extending = (topLength_target > current_length_top);
        //final double remaining_length_bottom_sign=Math.signum(remaining_length_bottom);

        //longer than the 
        if(remaining_length_bottom_magnitude < 3 && remaining_length_top_magnitude < 3){
        
        //if(true){
        /////////////////////////////////////////////////////////////////
        //
        //  exit logic
        //
        /////////////////////////////////////////////////////////////////

        // since we calculate everything here anyway, we check the exit condition and store it in a member variable

/*      if((remaining_length_bottom_magnitude <= allowed_error_in_length_to_finish_bottom ) &&
                (remaining_length_top_magnitude <= allowed_error_in_length_to_finish_top)){

            both_arms_are_at_target_value=true;
        }
        */

        /////////////////////////////////////////////////////////////////
        //
        //  step logic
        //
        /////////////////////////////////////////////////////////////////

        // Best case is the arms are close together so move them both
        if (Math.abs(delta_length) < Math.abs(max_delta_length))
        {
            step_bottom();
            step_top();
        }
        else   // the arms are too far apart, so we pause the one that is "ahead" so the other guy can catch up
        {
            if (remaining_length_bottom_magnitude < remaining_length_top_magnitude) {
                pause_bottom();
                step_top();

            } else {
                step_bottom();
                pause_top();
            }
        }

//        else{
//            System.out.println("Everything fine");
//        }
//        //System.out.println(intake.getBottomWinchLength() + " " + intake.getTopWinchLength());
//        */
        }
        else{

        /////////////////////////////////////////////////////////////////
        //
        //  step logic
        //
        /////////////////////////////////////////////////////////////////

        // the arms are too far apart, so we pause the one that is "ahead" so the other guy can catch up

        if((Math.abs(delta_length) > Math.abs(max_delta_length_when_seperated))){
            if (remaining_length_bottom_magnitude < remaining_length_top_magnitude) {
                pause_bottom();
                step_top();

            } else {
                step_bottom();
                pause_top();
            }
        }
        else if((is_overhung)){
                if(is_extending){
                    step_bottom();
                    pause_top();
                    System.out.println("Extending: " + is_extending);
                }
                else{
                    pause_bottom();
                    step_top();   
                    System.out.println("Extending: " + is_extending);
                }
            }
        else {// if((Math.abs(delta_length) < Math.abs(max_delta_length_when_seperated)) && Math.abs(min_delta_length_when_seperated) < Math.abs(delta_length))
            step_bottom();
            step_top();
            System.out.println(is_overhung);
        }

    }


    }


    public void end(boolean interrupted){
        intake.resetPID();
        System.out.println("Stopped");
    }

    public boolean isFinished() {

        /*if(both_arms_are_at_target_value) {
            if(innerTimer.get() > 0.2) return true;
        }
        else innerTimer.reset();
        */
       // /* 
        if((Math.abs(intake.getBottomWinchLength()-bottomLength_target)<1) && (intake.getTopWinchLength()-topLength_target)<1){
        return true;
        }
        else innerTimer.reset();

      //  */

        
        if(timer.get() > 2){
            return true;
        }
        

        return false;
      }


}
