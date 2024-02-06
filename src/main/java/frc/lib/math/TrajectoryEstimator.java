package frc.lib.math;

import edu.wpi.first.math.util.Units;
import frc.lib.helpers.OutputUnit;
import frc.lib.helpers.UnitTypes;

public class TrajectoryEstimator {
    /**
     * The horizontal shift of the shooter shaft with respect to the edge of the robot.
     */
    @OutputUnit(UnitTypes.METERS)
    private static final double x_s = 0.26;
    /**
     * The vertical shift of the shooter shaft with respect to the edge of the robot.
     */
    @OutputUnit(UnitTypes.METERS)
    private static final double y_s = 0.335;
    /**
     * The horizontal shift of the camera with respect to the edge of the robot.
     */
    @OutputUnit(UnitTypes.METERS)
    private static final double x_sl = 0.0; // TODO fill in correct value
    // TODO fill in correct value
    /**
     * The gravitational acceleration constant. Fixed.
     */
    @OutputUnit(UnitTypes.METERS_PER_SECOND_SQUARED)
    private static final double g = 9.8;
    /**
     * The initial shooting speed of the note upon leaving the shooter.
     */
    @OutputUnit(UnitTypes.METERS_PER_SECOND)
    private static final double v_0 = 18.4; // 12.0
    /**
     * The first order air drag constant of the note.
     * <b>DO NOT SET THIS TO ZERO!!!!</b>
     */
    @OutputUnit(UnitTypes.ONE_OVER_SECOND)
    private static final double b = 1.05; // 0.7
    /**
     * The angle of elevation of the shooter when the shaft encoder's value is 0. Should be close to 0.
     */
    @OutputUnit(UnitTypes.DEGREES)
    private static final double phi_0 = 1.5; // 6.35
    /**
     * The length from the shooter shaft to the shooter exit.
     */
    @OutputUnit(UnitTypes.METERS)
    private static final double l = 0.21; // 0.3

    public static double getAngleOfElevation1(double distanceFromCamera) {
        double prevError = Integer.MAX_VALUE, error;
        for (double outputAngle = 0.0; outputAngle <= 90.0; outputAngle++) {
            double exitX = distanceFromCamera - x_sl + x_s - l * Math.cos(Units.degreesToRadians(outputAngle + phi_0));
            double exitY = y_s + l * Math.sin(Units.degreesToRadians(outputAngle + phi_0));
            double y = q(0.2286, Units.degreesToRadians(outputAngle + phi_0), exitX, exitY);
            error = Math.abs(y - 2.0431125);
            if (error > prevError) {
                return outputAngle - 1;
            }

            prevError = error;
        }

        return 90.0;
    }

    private static double t(double x, double phi) {
        return -1.0 / b * Math.log(1.0 - b / v_0 / Math.cos(phi) * -x);
    }

    private static double y(double t, double phi) {
        return 1.0 / b * (g / b + v_0 * Math.sin(phi)) * (1.0 - Math.exp(-b * t)) - g / b * t;
    }

    private static double q(double x, double phi, double x_0, double y_0) {
        return y(t(x - x_0, phi), phi) + y_0;
    }
}
