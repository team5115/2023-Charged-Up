package frc.team5115.Classes.Software; 
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5115.Constants.*;

public class PhotonVision extends SubsystemBase{
    private PhotonCamera photonCameraL;
    private PhotonCamera photonCameraR;
    private PhotonPoseEstimator photonPoseEstimatorL;
    private PhotonPoseEstimator photonPoseEstimatorR;

    public PhotonVision() {
        photonCameraL = new PhotonCamera(VisionConstants.leftCameraName);
        photonCameraR = new PhotonCamera(VisionConstants.rightCameraName);
        ArrayList<AprilTag> aprilTagList = new ArrayList<AprilTag>();

        // Add all the april tags
        aprilTagList.add(GenerateAprilTag(1, +7.243604, -2.936589, 180));
        aprilTagList.add(GenerateAprilTag(2, +7.243604, -1.260189, 180));
        aprilTagList.add(GenerateAprilTag(3, +7.243604, +0.416211, 180));

        aprilTagList.add(GenerateAprilTag(8, -7.243604, -2.936589, 000));
        aprilTagList.add(GenerateAprilTag(7, -7.243604, -1.260189, 000));
        aprilTagList.add(GenerateAprilTag(6, -7.243604, +0.416211, 000));

        aprilTagList.add(GenerateAprilTag(4, +7.908830, +2.741613, 180));
        aprilTagList.add(GenerateAprilTag(5, -7.908830, +2.741613, 000));

        AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(aprilTagList, FieldConstants.length, FieldConstants.width);
        photonPoseEstimatorL = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, photonCameraL, VisionConstants.robotToCamL);
        photonPoseEstimatorR = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, photonCameraR, VisionConstants.robotToCamR);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        //Trusting the left camera more, no idea on how to use filters to get the most information out of both cameras 2-6-2022
        if(photonPoseEstimatorL.update().isPresent()) return photonPoseEstimatorL.update();
        if(photonPoseEstimatorR.update().isPresent()) return photonPoseEstimatorR.update();
        return Optional.empty();
    }

    private AprilTag GenerateAprilTag(int id, double x, double y, double rotationDegrees) {
        return new AprilTag( id, new Pose3d( new Pose2d( x, y, Rotation2d.fromDegrees(rotationDegrees))));
    }
}
