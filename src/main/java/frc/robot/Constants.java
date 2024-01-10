package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class SwerveModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kWheelCircumrerence = kWheelDiameterMeters * 2 * Math.PI;
        public static final double kDriveMotorGearRatio = 1. / 8.14;
        public static final double kTurningMotorGearRatio = 7. / 150.;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

        public static final double kPTurning = 0.75;
        public static final double kITurning = 0;
        public static final double kDTurning = 0;

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;

        public static final double kPathingX_kP = 0;
        public static final double kPathingX_kI = 0;
        public static final double kPathingX_kD = 0;

        public static final double kPathingY_kP = 0;
        public static final double kPathingY_kI = 0;
        public static final double kPathingY_kD = 0;

        public static final double kPathingTurning_kP = 0;
        public static final double kPathingTurning_kI = 0;
        public static final double kPathingTurning_kD = 0;

        public static double kCorrectPositionXController = 0;
        public static double kCorrectPositionYController = 0;
        public static double kCorrectPositionThetaController = 0;

        public static final TrapezoidProfile.Constraints kDriveControllerConstraints = //
            new TrapezoidProfile.Constraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAccelerationMetersPerSecondSquared);
        
    }

    public static final class DriveConstants {
        public static final boolean fieldOriented = true;

        public static final double kTrackWidth = Units.inchesToMeters(24);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(24);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(kWheelBase / 2, -kTrackWidth / 2), new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.698 * 180 / Math.PI;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 1.803 * 180 / Math.PI;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.059 * 180 / Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.645 * 180 / Math.PI;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 3;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 1.5 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class VisionConstants {
        public static final Transform3d Tag2Goal =
        new Transform3d(
                new Translation3d(-DriveConstants.kTrackWidth / 2, 0, 0),
                new Rotation3d(0, 0, 0));

        public static final double klimelightLensHeightInches = Units.metersToInches(43 / 100);
        public static final double kgoalHeightInches = Units.metersToInches(68.9 / 100);
        public static final double klimelightMountAngleDegrees = 0;
        public static final double klimelightHorizontalOffsetInches = Units.metersToInches(26.9 / 100); // lenght from limelight to the center of robot

        public static final double kAutoTrackYP = 0.4;

        public static double kAutoTrackThetaP = 0.01;

        // public static final double kAutotrackMaxSpeed = 1.0; //use tanh
        // public static final double kAutotrackIncreaseSpeed = 0.7; //use tanh
        // public static final double kAutotrackRotationIncreaseSpeed = 0.15; //use tanh


    }
}
