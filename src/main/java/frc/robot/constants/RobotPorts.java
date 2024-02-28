package frc.robot.constants;

import frc.lib.helpers.TidiedUp;

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
@TidiedUp
public class RobotPorts {
    public enum CAN {
        FL_DRIVE(2), FR_DRIVE(4), BL_DRIVE(5), BR_DRIVE(7),
        FL_STEER(1), FR_STEER(3), BL_STEER(6), BR_STEER(8),
        FL_ABS_ENCODER(9), FR_ABS_ENCODER(10), BL_ABS_ENCODER(11), BR_ABS_ENCODER(12),
        RIGHT_SHOOTER(15), LEFT_SHOOTER(14),
        LEFT_INTAKE(18), RIGHT_INTAKE(17), LEFT_INTAKE_LIFTER(20), RIGHT_INTAKE_LIFTER(19),
        LEFT_TELESCOPE(21), RIGHT_TELESCOPE(22);

        private final int port;

        CAN(int port) {
            this.port = port;
        }

        public int get() {
            return this.port;
        }
    }

    public enum DIO {
        LIFTER_ENCODER(1);

        private final int port;

        DIO(int port) {
            this.port = port;
        }

        public int get() {
            return this.port;
        }
    }

    public enum PWM {
        LEFT_TELESCOPE_LED_PORT(0), RIGHT_TELESCOPE_LED_PORT(1);

        private final int port;

        PWM(int port) {
            this.port = port;
        }

        public int get() {
            return this.port;
        }
    }
}
