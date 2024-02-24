package frc.robot.constants;

import java.util.Arrays;

/**
 * Here are some abbreviations used in this enum:
 * <ul>
 *     <li><b>FL</b> stands for <b>Front Left</b> swerve module</li>
 *     <li><b>FR</b> stands for <b>Front Right</b> swerve module</li>
 *     <li><b>BL</b> stands for <b>Back Left</b> swerve module</li>
 *     <li><b>BR</b> stands for <b>Back Right</b> swerve module</li>
 *     <li><b>ABS</b> stands for <b>Absolute</b></li>
 * </ul>
 */
public enum RobotCANPorts {
    FL_DRIVE(2), FR_DRIVE(4), BL_DRIVE(5), BR_DRIVE(7),
    FL_STEER(1), FR_STEER(3), BL_STEER(6), BR_STEER(8),
    FL_ABS_ENCODER(9), FR_ABS_ENCODER(10), BL_ABS_ENCODER(11), BR_ABS_ENCODER(12),
    RIGHT_SHOOTER(15), LEFT_SHOOTER(14), SLOPE_TOGGLER(16),
    LEFT_INTAKE(18), RIGHT_INTAKE(17), LEFT_INTAKE_LIFTER(20), RIGHT_INTAKE_LIFTER(19),
    LEFT_TELESCOPE(21), RIGHT_TELESCOPE(22);

    private final int port;

    RobotCANPorts(int port) {
        this.port = port;
    }

    public static boolean anyMatch(int port, RobotCANPorts... objects) {
        return Arrays.stream(objects).anyMatch(obj -> obj.get() == port);
    }

    public int get() {
        return this.port;
    }
}
