package frc.lib.math;

import frc.lib.helpers.OutputUnit;
import frc.lib.helpers.UnitTypes;

public final class TrajectoryConstants {
    /**
     * The horizontal shift of the shooter shaft with respect to the edge of the robot.
     */
    @OutputUnit(UnitTypes.METERS)
    public static final double x_s = 0.26;

    /**
     * The vertical shift of the shooter shaft with respect to the edge of the robot.
     */
    @OutputUnit(UnitTypes.METERS)
    public static final double y_s = 0.335;

    /**
     * The horizontal shift of the camera with respect to the edge of the robot.
     */
    @OutputUnit(UnitTypes.METERS)
    public static final double x_sl = 0.0; // TODO fill in correct value

    /**
     * The gravitational acceleration constant. Fixed.
     */
    @OutputUnit(UnitTypes.METERS_PER_SECOND_SQUARED)
    public static final double g = 9.8;

    /**
     * The initial shooting speed of the note upon leaving the shooter.
     */
    @OutputUnit(UnitTypes.METERS_PER_SECOND)
    public static final double v_0 = 18.4; // 12.0

    /**
     * The second order air drag constant of the note.
     * Can be set to zero. In this case, the second order air drag is ignored.
     */
    @OutputUnit(UnitTypes.ONE_OVER_METER)
    public static final double a = 0.0;

    /**
     * The first order air drag constant of the note.
     * Can be set to zero. In this case, the first order air drag is ignored.
     */
    @OutputUnit(UnitTypes.ONE_OVER_SECOND)
    public static final double b = 1.05; // 0.7

    /**
     * The angle of elevation of the shooter when the shaft encoder's value is 0. Should be close to 0.
     */
    @OutputUnit(UnitTypes.DEGREES)
    public static final double phi_0 = 1.5; // 6.35

    /**
     * The length from the shooter shaft to the shooter exit.
     */
    @OutputUnit(UnitTypes.METERS)
    public static final double l = 0.21; // 0.3
}
