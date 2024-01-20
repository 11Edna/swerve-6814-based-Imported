package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(1.5);
        public static final double kDriveMotorGearRatio = 1 / 8.14;
        public static final double kTurningMotorGearRatio = 1 / 21.43;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.6;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(23);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(23);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final Translation2d flModuleOffset = new Translation2d(0.4, 0.4);
        public static final Translation2d frModuleOffset = new Translation2d(0.4, -0.4);
        public static final Translation2d blModuleOffset = new Translation2d(-0.4, 0.4);
        public static final Translation2d brModuleOffset = new Translation2d(-0.4, -0.4);

        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kBackLeftDriveMotorPort = 1;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackRightDriveMotorPort = 4;

        public static final int kFrontLeftTurningMotorPort = 6;
        public static final int kBackLeftTurningMotorPort = 5;
        public static final int kFrontRightTurningMotorPort = 7;
        public static final int kBackRightTurningMotorPort = 8;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 0;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 6.26;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 4.851;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 5.923;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 2.921;

        public static final double speedMultiplier = .5;
        public static final double turnSpeedMultiplier = .6;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 3.81;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * speedMultiplier;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond * turnSpeedMultiplier;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    }

    public static final class AutoConstants {
        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(9.0, 0, 0), // Translation constants 
        new PIDConstants(1.1, 0, 0), // Rotation constants 
        DriveConstants.kTeleDriveMaxSpeedMetersPerSecond, 
        DriveConstants.kWheelBase/2, // Drive base radius (distance from center to furthest module) 
        new ReplanningConfig());
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }

}
