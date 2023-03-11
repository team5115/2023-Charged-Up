package frc.team5115.Classes.Software; 
import java.lang.StackWalker.Option;
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
    private PhotonCamera photonCameraLeft;
    private PhotonCamera photonCameraRight;
    private PhotonPoseEstimator photonPoseEstimatorLeft;
    private PhotonPoseEstimator photonPoseEstimatorRight;

    public PhotonVision() {
        photonCameraLeft = new PhotonCamera(VisionConstants.leftCameraName);
        photonCameraRight = new PhotonCamera(VisionConstants.rightCameraName);
        ArrayList<AprilTag> aprilTagList = new ArrayList<AprilTag>();

        // Add all the april tags

        //Red Grid
        aprilTagList.add(GenerateAprilTag(1, +7.243604, -2.936589, 180));
        aprilTagList.add(GenerateAprilTag(2, +7.243604, -1.260189, 180));
        aprilTagList.add(GenerateAprilTag(3, +7.243604, +0.416211, 180));

        //Blue Grid
        aprilTagList.add(GenerateAprilTag(8, -7.243604, -2.936589, 000));
        aprilTagList.add(GenerateAprilTag(7, -7.243604, -1.260189, 000));
        aprilTagList.add(GenerateAprilTag(6, -7.243604, +0.416211, 000));

        //Substations
        aprilTagList.add(GenerateAprilTag(4, +7.908830, +2.741613, 180));
        aprilTagList.add(GenerateAprilTag(5, -7.908830, +2.741613, 000));

        AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(aprilTagList, FieldConstants.length, FieldConstants.width);
        photonPoseEstimatorLeft = new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT, photonCameraLeft, VisionConstants.robotToLeftCam);
        photonPoseEstimatorRight = new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT, photonCameraRight, VisionConstants.robotToRightCam);
    }

    public void Update() {
        Optional<EstimatedRobotPose> result = getEstimatedPose();

        if (result.isPresent()) {
            System.out.println(result.get().estimatedPose.toString());
        } else {
            System.out.println(result);
        }
    }

    private Optional<EstimatedRobotPose> getEstimatedPose() {
        Optional<EstimatedRobotPose> leftResult = photonPoseEstimatorLeft.update();
        Optional<EstimatedRobotPose> rightResult = photonPoseEstimatorRight.update();
        if (leftResult.isPresent()) {
            return leftResult;
        }
        if (rightResult.isPresent()) {
            return rightResult;
        }
        return Optional.empty();
    }

    private AprilTag GenerateAprilTag(int id, double x, double y, double rotationDegrees) {
        return new AprilTag( id, new Pose3d( new Pose2d( x, y, Rotation2d.fromDegrees(rotationDegrees))));
    }
}