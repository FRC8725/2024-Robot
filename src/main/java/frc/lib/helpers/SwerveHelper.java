package frc.lib.helpers;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * Helper for swerve drive.
 */
public final class SwerveHelper {
    /**
     * Construct {@link SwerveDriveKinematics} with the order of FL, FR, BL, BR in robot coordination system.
     * See {@link frc.robot.constants.RobotCANPorts} for introduction to abbreviations.
     * @param trackWidth the left-right distance between two swerve modules.
     * @param wheelBase the front-back distance between two swerve modules.
     * @return a {@link SwerveDriveKinematics} object of the swerve subsystem.
     */
    @CoordinateSystem(CoordinationPolicy.ROBOT_COORDINATION)
    public static SwerveDriveKinematics constructKinematics(double trackWidth, double wheelBase) {
        return new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        );
    }
}
