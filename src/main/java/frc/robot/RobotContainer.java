package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;



public class RobotContainer {
    private final Vision vision = new Vision();
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(vision);
    private final PoseEstimator poseEstimator = new PoseEstimator(swerveSubsystem, vision, new Pose2d(2, 7, swerveSubsystem.getRotation2d()));
    private AutoCommand autoComands = new AutoCommand(vision, swerveSubsystem);
    private final CommandXboxController driverJoytick = new CommandXboxController(OIConstants.kDriverControllerPort);
    private final Joystick buttonBox = new Joystick(1);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        //CameraServer.startAutomaticCapture();
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.b().getAsBoolean()));


        configureButtonBindings();

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("StraightAuto", autoChooser);
        SmartDashboard.putData("curvyAuto", autoChooser);


    }

    private void configureButtonBindings() {
        PathPlannerPath spin = PathPlannerPath.fromPathFile("Spin");
        driverJoytick.leftBumper().onTrue(swerveSubsystem.zeroHeadingCommand());
        driverJoytick.rightBumper().onTrue(AutoBuilder.followPath(spin));
        //cool spin move
        new JoystickButton(buttonBox, 2).onTrue(Commands.runOnce(() -> {
            //get current pose
            Pose2d currentPose = swerveSubsystem.getPose();

            //make start, mid, and end pose
            Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
            Pose2d midPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(1.0, 0.5)), new Rotation2d());
            Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, midPos, endPos);
            PathPlannerPath path = new PathPlannerPath(
                bezierPoints, 
                new PathConstraints(
                3.81, 2.0, 
                Units.degreesToRadians(360), Units.degreesToRadians(540)
                ),  
                new GoalEndState(0.0, Rotation2d.fromDegrees(0))
            );

            // Prevent this path from being flipped on the red alliance, since the given positions are already correct
            path.preventFlipping = true;

            AutoBuilder.followPath(path).schedule();
            }));

            new JoystickButton(buttonBox, 3).onTrue(autoComands.toNote());
            new JoystickButton(buttonBox, 4).onTrue(autoComands.PathToPose(1, 1, 0));
        }

    public Command getAutonomousCommand() {
        swerveSubsystem.zeroHeading();
        return autoChooser.getSelected();
  }
}