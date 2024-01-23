package frc.lib.helpers;

@SuppressWarnings("unused")
public enum CoordinationPolicy {
    /**
     * The coordinate system used on {@link edu.wpi.first.wpilibj.XboxController}'s raw axis.
     * Forward for +x and leftward for +y.
     */
    XBOX_COORDINATION,

    /**
     * The coordinate system used on the robot.
     * Forward for +x, leftward for +y, and counterclockwise for +theta.
     * see <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html">Wpilib Coordinate System</a>
     * for more info.
     */
    ROBOT_COORDINATION,

    /**
     * The coordinate system used on the field.
     * Rightward for +x, forward for +y, and counterclockwise for +theta.
     * see <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html">Wpilib Coordinate System</a>
     * for more info.
     */
    FIELD_COORDINATION
}
