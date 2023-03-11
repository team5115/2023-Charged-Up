package frc.team5115;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants{

    public static final boolean MECANUM = false; 
  
    //motor ids
    public static final byte FRONT_LEFT_MOTOR_ID = 1;
    public static final byte FRONT_RIGHT_MOTOR_ID = 2;
    public static final byte BACK_LEFT_MOTOR_ID = 3;
    public static final byte BACK_RIGHT_MOTOR_ID = 4;

    public static final double TALON_ENCODER_CALIBRATION = (63.837/4104.5);
    public static final double NEO_ENCODER_CALIBRATION = (0.3192*10.71/60);
    public static final double MAX_VOLTAGE = 12;

    //X-Box
    public static final byte JOY_X_AXIS_ID = 0;
    public static final byte JOY_Y_AXIS_ID = 1;
    public static final byte JOY_Z_AXIS_ID = 4; 

    // NEW Feedforward
    public static final double kS = 0.18296; 
    public static final double kV = 4.2023;
    public static final double kA = 0.28613;

    public static final double DRIVE_MOTOR_MAX_VOLTAGE = 12;
    // distance between the left wheels and the right wheels in meters
    // 0.70 for wide, 0.57 for long
    public static final double TRACKING_WIDTH_METERS = 0.70;

    public static final double TARGET_ANGLE = 1;

    // Photon vision constants    
    public static class FieldConstants {
        public static final double length = Units.feetToMeters(54);
        public static final double width = Units.feetToMeters(27);
        public static final double startX = -7;
        public static final double startY = -1;
        public static final double startAngleDeg = 0;
        public static final Rotation2d startAngle = Rotation2d.fromDegrees(startAngleDeg);
    }

     public static class VisionConstants {
        public static final String leftCameraName = "HD_USB_Camera";
        public static final String rightCameraName = "Microsoft_LifeCam_HD-3000";
        public static final Transform3d robotToLeftCam = new Transform3d( new Translation3d(0.0, 0.0, 0.0), new Rotation3d( 0, 0, 0));
        public static final Transform3d robotToRightCam = new Transform3d( new Translation3d(0.0, 0.0, 0.0), new Rotation3d( 0, 0, 0));

    }

}