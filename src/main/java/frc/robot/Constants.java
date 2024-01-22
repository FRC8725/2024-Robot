package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = 1.5 * Math.PI;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 2;

        public static final double kPathing_kP = 5.0;
        public static final double kPathing_kI = 0.05;
        public static final double kPathing_kD = 0.;

        public static final double kPathingTurning_kP = 5.0;
        public static final double kPathingTurning_kI = 0.;
        public static final double kPathingTurning_kD = 0.;

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
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2), new Translation2d(-kWheelBase / 2, kTrackWidth / 2), new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final boolean kFrontLeftTurningMotorReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveMotorReversed = true;
        public static final boolean kBackLeftDriveMotorReversed = true;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kBackRightDriveMotorReversed = false;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetDegrees = Units.radiansToDegrees(kFrontLeftDriveAbsoluteEncoderOffsetRad);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetDegrees = Units.radiansToDegrees(kBackLeftDriveAbsoluteEncoderOffsetRad);
        public static final double kFrontRightDriveAbsoluteEncoderOffsetDegrees = Units.radiansToDegrees(kFrontRightDriveAbsoluteEncoderOffsetRad);
        public static final double kBackRightDriveAbsoluteEncoderOffsetDegrees = Units.radiansToDegrees(kBackRightDriveAbsoluteEncoderOffsetRad);

        public static final double kPhysicalMaxSpeedMetersPerSecond = 3;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = 2 / kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                2 * Math.PI / kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class VisionConstants {
        public static final Transform3d Tag2Goal = new Transform3d(new Translation3d(-DriveConstants.kTrackWidth / 2, 0, 0), new Rotation3d(0, 0, 0));

        public static final double klimelightHeightcm = 45.0;
        public static final double klimelightMountDegrees = 0;

        public static final double knoteHeight = 0;

        public static final double kAutoTrackYP = 0.01;
        public static final double kAutoTrackThetaP = 0.03;

    }

    public static final class ShooterConstants {
        public static final double kshootSpeed = 0.9;
        public static final double kloadSpeed = 0.5;
    }
}
