package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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


        public static final TrapezoidProfile.Constraints kDriveControllerConstraints = //
                new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCELERATION);

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ACCELERATION);

    }

    public static final class DriveConstants {
        @OutputUnit(UnitTypes.METERS_PER_SECOND)
        public static final double TELEOP_MAX_ROBOT_SPEED = 2.0;
        @OutputUnit(UnitTypes.RADIANS_PER_SECOND)
        public static final double TELEOP_MAX_ROBOT_ANGULAR_SPEED = Math.PI;
        @OutputUnit(UnitTypes.METERS_PER_SECOND_SQUARED)
        public static final double TELEOP_MAX_ACCELERATION = 6.0;
        @OutputUnit(UnitTypes.RADIANS_PER_SECOND_SQUARED)
        public static final double TELEOP_MAX_ANGULAR_ACCELERATION = 2.0 * Math.PI;
        @OutputUnit(UnitTypes.METERS_PER_SECOND)
        public static final double AUTO_MAX_ROBOT_SPEED = 2.0;
        @OutputUnit(UnitTypes.RADIANS_PER_SECOND)
        public static final double AUTO_MAX_ROBOT_ANGULAR_SPEED = Math.PI;
        @OutputUnit(UnitTypes.METERS_PER_SECOND_SQUARED)
        public static final double AUTO_MAX_ACCELERATION = 6.0;
        @OutputUnit(UnitTypes.RADIANS_PER_SECOND_SQUARED)
        public static final double AUTO_MAX_ANGULAR_ACCELERATION = 2.0 * Math.PI;
    }
}
