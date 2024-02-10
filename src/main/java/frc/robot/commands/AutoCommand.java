package frc.robot.commands;

import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;


public class AutoCommand {
        private final Vision vision;
        private final SwerveSubsystem swerveSubsystem;

    public AutoCommand(Vision vision, SwerveSubsystem swerveSubsystem){
        this.vision = vision;
        this.swerveSubsystem = swerveSubsystem;
    }

    public Command PathToPose( double xPos, double yPos, double rotation){
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d targetPose = new Pose2d(xPos, yPos, Rotation2d.fromDegrees(rotation));

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond, DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    
        );
        return pathfindingCommand;
    }
    
    public Command toAmp(){
        Pose2d targetPose = new Pose2d(0, 0, Rotation2d.fromDegrees(90));

        PathConstraints constraints = new PathConstraints(
                DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond, DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    
        );
        return pathfindingCommand;

    }

    public Command amp() {
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d targetPose = new Pose2d(14.70, 7.65, Rotation2d.fromDegrees(90));

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                2, 2.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        return pathfindingCommand;
    }

    public Command toNote(){
        Pose2d currentPose = swerveSubsystem.getPose();
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d targetPose = new Pose2d(currentPose.getTranslation().plus(new Translation2d(vision.getNoteDistance(), 0.0)), new Rotation2d());

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.81, 2.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        return pathfindingCommand;
    }
}
