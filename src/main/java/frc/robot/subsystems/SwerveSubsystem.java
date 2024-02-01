package frc.robot.subsystems;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.Volts;

import java.beans.DesignMode;
import java.io.PipedInputStream;
import java.util.ArrayList;


public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(13);
    //private final Field2d m_field = new Field2d();
    private final Vision vision;
    SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, getRotation2d(),
        new SwerveModulePosition[] {
        frontLeft.getPosition(),
        backLeft.getPosition(),
        frontRight.getPosition(),
        backRight.getPosition()
    }, new Pose2d(2.0, 7.0, new Rotation2d()));

    public SwerveSubsystem(Vision vision) {
        this.vision = vision;
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
                resetOdometry(getPose());
            } catch (Exception e) { System.out.println("Exception in auto: 'SwerveSubsystem()'");
            }
        }).start();

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            //m_field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            //m_field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            //m_field.getObject("path").setPoses(poses);
        });

        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdometry, 
            this::getSpeeds, 
            this::driveRobotRelative, 
            Constants.AutoConstants.pathFollowerConfig, 
            () -> true, 
            this);
    }

    public ChassisSpeeds getChassisSpeed() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(), backLeft.getState(),
            frontRight.getState(),
            backRight.getState());
      }

    public void zeroHeading() {
        gyro.reset();
    }

    public Command zeroHeadingCommand() {
        return this.runOnce(() -> this. gyro.reset());
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d().times(-1);
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public Pose2d getAutoPose() {
        updateOdometery();
        return odometer.getPoseMeters();
      }

      public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = frontLeft.getState();
        states[1] = backLeft.getState();
        states [2] = frontRight.getState();
        states[3] = backRight.getState();
        return states;
      }

      public ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
      }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            backLeft.getPosition(),
            frontRight.getPosition(),
            backRight.getPosition()
        }, pose //new Rotation2d()
        );
    }

    public SwerveModulePosition [] getPosition(){
        return new SwerveModulePosition[] {frontLeft.getPosition(), backLeft.getPosition(), frontRight.getPosition(),
            backRight.getPosition()};
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
      }
 
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }
    
    public void updateOdometery(){
        odometer.update(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            backLeft.getPosition(),
            frontRight.getPosition(),
            backRight.getPosition()
        });
    }

    @Override
    public void periodic() {
        updateOdometery();
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("FLposition", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BLposition", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("FRposition", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BRposition", backRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("AbsoluteEnc Voltage FL", frontLeft.getEncoder().getVoltage());
        SmartDashboard.putNumber("AbsoluteEnc Voltage BL", backLeft.getEncoder().getVoltage());
        SmartDashboard.putNumber("AbsoluteEnc Voltage FR", frontRight.getEncoder().getVoltage());
        SmartDashboard.putNumber("AbsoluteEnc Voltage BR", backRight.getEncoder().getVoltage());
        SmartDashboard.putNumber("Encoder position FL", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("Encoder position FR", frontRight.getTurningPosition());
        SmartDashboard.putNumber("Encoder position BL", backLeft.getTurningPosition());
        SmartDashboard.putNumber("Encoder position BR", backRight.getTurningPosition());
        //SmartDashboard.putData("Field", m_field);
        //m_field.setRobotPose(odometer.getPoseMeters());
        vision.getEstimatedGlobalPose(getPose());
        //updatePose();

    }

    public void stopModules() {
        frontLeft.stop();
        backLeft.stop();
        frontRight.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        backLeft.setDesiredState(desiredStates[1]);
        frontRight.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    // public void setModuleStatesAuto(SwerveModuleState[] desiredStates) {
    //     //SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    //     frontLeft.setDesiredStateAuto(desiredStates[0]);
    //     backLeft.setDesiredStateAuto(desiredStates[1]);
    //     frontRight.setDesiredStateAuto(desiredStates[2]);
    //     backRight.setDesiredStateAuto(desiredStates[3]);
    // }

}