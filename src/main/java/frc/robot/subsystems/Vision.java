package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonVersion;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonCamera camera = new PhotonCamera("photonvision");

    Transform3d robotToCam = new Transform3d(new Translation3d(-0.305, .3302, 0.305), new Rotation3d(0,0,45));
    PhotonPoseEstimator photonPoseEstimator; 
     private final ArrayList<double[]> poses = new ArrayList<>();
    public Vision(){
    try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public boolean valid() {
        var result = camera.getLatestResult();
        boolean foundTarget = result.hasTargets();
        return foundTarget;
    }


    //  public void updatePose() {
    //     var result = camera.getLatestResult();
    //     boolean hasTargets = result.hasTargets();
    //     PhotonTrackedTarget target = result.getBestTarget();
    //     if (hasTargets == true){
    //         Transform3d poseTag = target.getBestCameraToTarget();
    //         Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);
    //     }
    // }

     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public double[] getLatestPose3d() {
        return poses.size() == 0 ? new double[7] : poses.remove(0);
    }

    public double getNoteDistance(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        double limelightMountAngleDegrees = 23.0; 

        double limelightLensHeightCm = 41; 

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        double distanceFromLimelightToGoalCm = (limelightLensHeightCm) / Math.tan(angleToGoalRadians);
        SmartDashboard.putNumber("Note Distance Cm", distanceFromLimelightToGoalCm);;

        return distanceFromLimelightToGoalCm;
    }
}
