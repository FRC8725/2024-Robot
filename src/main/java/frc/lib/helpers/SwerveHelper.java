package frc.lib.helpers;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * Helper for swerve drive.
 */
public final class SwerveHelper {
    /**
     * A helper enum for {@link SwerveDriveKinematics}' output indexes. the order of the indexes should be
     * the same as the order of the {@link Translation2d}s in {@link SwerveHelper#constructKinematics(double, double)}
     */
    public enum ModuleIds {
        FL(0), FR(1), BL(2), BR(3);

        private final int id;

        ModuleIds(int id) {
            this.id = id;
        }

        public int get() {
            return this.id;
        }
    }

    /**
     * Construct {@link SwerveDriveKinematics} with the order of <b>FL -> FR -> BL -> BR</b> in robot coordination system.
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
