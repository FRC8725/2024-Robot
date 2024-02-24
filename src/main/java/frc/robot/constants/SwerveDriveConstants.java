package frc.robot.constants;

import frc.lib.helpers.OutputUnit;
import frc.lib.helpers.UnitTypes;

public final class SwerveDriveConstants {
    @OutputUnit(UnitTypes.METERS_PER_SECOND)
    public static final double TELEOP_MAX_ROBOT_SPEED = 3.0; // 2.0    @OutputUnit(UnitTypes.RADIANS_PER_SECOND)
    public static final double TELEOP_MAX_ROBOT_ANGULAR_SPEED = 1 * Math.PI; // Math.PI
    @OutputUnit(UnitTypes.METERS_PER_SECOND_SQUARED)
    public static final double TELEOP_MAX_ACCELERATION = TELEOP_MAX_ROBOT_SPEED * 3.0;
    @OutputUnit(UnitTypes.RADIANS_PER_SECOND_SQUARED)
    public static final double TELEOP_MAX_ANGULAR_ACCELERATION = TELEOP_MAX_ROBOT_ANGULAR_SPEED * 2.0;
    @OutputUnit(UnitTypes.METERS_PER_SECOND)
    public static final double AUTO_MAX_ROBOT_SPEED = 2.0;
    @OutputUnit(UnitTypes.RADIANS_PER_SECOND)
    public static final double AUTO_MAX_ROBOT_ANGULAR_SPEED = Math.PI;
    @OutputUnit(UnitTypes.METERS_PER_SECOND_SQUARED)
    public static final double AUTO_MAX_ACCELERATION = AUTO_MAX_ROBOT_SPEED * 3.0;
    @OutputUnit(UnitTypes.RADIANS_PER_SECOND_SQUARED)
    public static final double AUTO_MAX_ANGULAR_ACCELERATION = 2.0 * Math.PI;
}
