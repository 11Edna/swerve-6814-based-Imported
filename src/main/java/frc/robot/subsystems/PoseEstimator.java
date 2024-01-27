package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class PoseEstimator extends SubsystemBase {
    private final Vision vision;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveSubsystem swerve;


    private final Field2d field = new Field2d();

    private boolean m_initializedPose = true;

    public PoseEstimator(SwerveSubsystem swerveSubsystem, Vision photon, Pose2d intialPose) {
        swerve = swerveSubsystem;
        vision = photon;
        SmartDashboard.putData("Field", field);
        SmartDashboard.putBoolean("Set Pose Est", false);

        m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, swerve.getRotation2d(), swerve.getPosition(), intialPose, VecBuilder.fill(0.229, 0.229, 0.0), VecBuilder.fill(5, 5, 5));
    }

    @Override
    public void periodic() {
        updatePoseEstimator();
        updateShuffleboard();
    }

    private void updatePoseEstimator() {
        Optional<EstimatedRobotPose> visionMeasurement = vision.getEstimatedGlobalPose(swerve.getPose());
        double velocity = Math.sqrt(swerve.getChassisSpeed().vxMetersPerSecond*swerve.getChassisSpeed().vxMetersPerSecond + swerve.getChassisSpeed().vyMetersPerSecond*swerve.getChassisSpeed().vyMetersPerSecond);
        double angularVelocity = swerve.getChassisSpeed().omegaRadiansPerSecond;
        Pose2d currentPose = getPose();

            m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), swerve.getRotation2d(), swerve.getPosition());
            //if (((currentPose.getTranslation().getDistance(visionPose.getTranslation()) <= VisionConstants.kPoseErrorAcceptance || !m_initializedPose) && visionMeasurement != new double[7] && visionPose.getTranslation().getDistance(currentPose.getTranslation()) >= 0.05 && velocity <= 3.0 && angularVelocity <= 0.5 * Math.PI && visionPose.getTranslation().getX() <= 5.0)) {
               // SmartDashboard.putBoolean("Set Pose Est", false);
                if (m_initializedPose) {
                    if(visionMeasurement.isPresent()){
                        var measure = visionMeasurement.get();


                        m_poseEstimator.addVisionMeasurement(measure.estimatedPose.toPose2d(), measure.timestampSeconds, VecBuilder.fill(5.0, 5.0, 5.0));
                    }
                } else {
                   // m_poseEstimator.addVisionMeasurement(visionPose, timestamp, VecBuilder.fill(0.0, 0.0, 999999));
                    //m_initializedPose = true;
                }
            }
        //}
    
        private void updateShuffleboard() {
            Pose2d pose = getPose();
            double[] poseArray = {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
            //SmartDashboard.putNumberArray("Robot Pose", poseArray);
            //SmartDashboard.putNumber("PoseEst X", poseArray[0]);
            //SmartDashboard.putNumber("PoseEst Y",  poseArray[1]);
            //SmartDashboard.putNumber("PoseEst Gyro",  poseArray[2]);
            field.setRobotPose(pose);
         }
    
        public Pose2d getPose() {
            return m_poseEstimator.getEstimatedPosition();
        }
    
        public Pose2d getPose(boolean allianceOrient) {
            return m_poseEstimator.getEstimatedPosition();
        }
    
        public boolean inside(Translation2d[] bounds, boolean onEdge) {
            Pose2d currentPose = getPose();
            double xMin = Math.min(bounds[0].getX(), bounds[1].getX());
            double xMax = Math.max(bounds[0].getX(), bounds[1].getX());
            double yMin = Math.min(bounds[0].getY(), bounds[1].getY());
            double yMax = Math.max(bounds[0].getY(), bounds[1].getY());
            return (
                (currentPose.getX() > xMin && currentPose.getX() < xMax) || (onEdge && (currentPose.getX() >= xMin && currentPose.getX() <= xMax)) 
                && 
                (currentPose.getY() > yMin && currentPose.getY() < yMax) || (onEdge && (currentPose.getY() >= yMin && currentPose.getY() <= yMax))
            );
        }
    
        public void resetOdometry(Pose2d pose) {
            swerve.resetOdometry(new Pose2d(pose.getTranslation(), pose.getRotation()));
            m_poseEstimator.resetPosition(swerve.getRotation2d().times(-1.0), swerve.getPosition(), pose);
        }
}
