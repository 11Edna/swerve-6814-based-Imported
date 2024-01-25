package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;

public class AutoCommand {
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
}
