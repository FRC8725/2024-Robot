package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.helpers.OutputUnit;
import frc.lib.helpers.UnitTypes;

public final class Constants {
    public static final class AutoConstants {
        @OutputUnit(UnitTypes.METERS_PER_SECOND)
        public static final double MAX_SPEED = 2;
        @OutputUnit(UnitTypes.RADIANS_PER_SECOND)
        public static final double MAX_ANGULAR_SPEED = 1.5 * Math.PI;
        @OutputUnit(UnitTypes.METERS_PER_SECOND_SQUARED)
        public static final double MAX_ACCELERATION = 2;
        @OutputUnit(UnitTypes.RADIANS_PER_SECOND_SQUARED)
        public static final double MAX_ANGULAR_ACCELERATION = 2;

        public static final double kPathing_kP = 5.0;
        public static final double kPathing_kI = 0.05;
        public static final double kPathing_kD = 0.;

        public static final double kPathingTurning_kP = 5.0;
        public static final double kPathingTurning_kI = 0.;
        public static final double kPathingTurning_kD = 0.;

        public static final TrapezoidProfile.Constraints kDriveControllerConstraints = //
                new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCELERATION);

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ACCELERATION);

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

        @OutputUnit(UnitTypes.METERS_PER_SECOND)
        public static final double PHYSICAL_MAX_SPEED = 4.0;
        @OutputUnit(UnitTypes.RADIANS_PER_SECOND)
        public static final double PHYSICAL_MAX_ANGULAR_SPEED = 3.0 * Math.PI;
        @OutputUnit(UnitTypes.METERS_PER_SECOND)
        public static final double TELEOP_MAX_SPEED = 2.0;
        @OutputUnit(UnitTypes.RADIANS_PER_SECOND)
        public static final double TELEOP_MAX_ANGULAR_SPEED = Math.PI;
        @OutputUnit(UnitTypes.METERS_PER_SECOND_SQUARED)
        public static final double TELEOP_MAX_ACCELERATION = 6.0;
        @OutputUnit(UnitTypes.RADIANS_PER_SECOND_SQUARED)
        public static final double TELEOP_MAX_ANGULAR_ACCELERATION = 2.0 * Math.PI;
    }

    public static final class VisionConstants {
        public static final Transform3d Tag2Goal = new Transform3d(new Translation3d(-DriveConstants.kTrackWidth / 2, 0, 0), new Rotation3d(0, 0, 0));

        @OutputUnit(UnitTypes.CENTIMETERS)
        public static final double kLimelightHeight = 45.0;
        @OutputUnit(UnitTypes.DEGREES)
        public static final double kLimelightMount = 0;
        public static final double kNoteHeight = 0;

        public static final double kAutoTrackYP = 0.01;
        public static final double kAutoTrackThetaP = 0.03;
    }

    public static final class ShooterConstants {
        public static final double kShootSpeed = 0.9;
        public static final double kLoadSpeed = 0.5;
    }

    public static final class  ElevatorConstants {
        public static final double kHeightLimit = 0.0;
        public static final double kSpeed = 0.5;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }
}
