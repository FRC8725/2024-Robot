package frc.lib.math;

import edu.wpi.first.math.util.Units;
import org.apache.commons.math3.exception.NumberIsTooSmallException;
import org.apache.commons.math3.ode.nonstiff.DormandPrince853Integrator;
import org.apache.commons.math3.util.FastMath;

public class TrajectoryEstimator {
    /**
     * The horizontal shift of the shooter shaft with respect to the edge of the robot.
     */
    @OutputUnit(UnitTypes.METERS)
    private static final double x_s = 0.165;
    /**
     * The vertical shift of the shooter shaft with respect to the edge of the robot.
     */
    @OutputUnit(UnitTypes.METERS)
    private static final double y_s = 0.345;
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
    private static final double v_0 = 11.4; // 12.0
    /**
     * The first order air drag constant of the note.
     * <b>DO NOT SET THIS TO ZERO!!!!</b>
     */
    @OutputUnit(UnitTypes.ONE_OVER_SECOND)
    private static final double b = 0.69; // 0.7
    /**
     * The angle of elevation of the shooter when the shaft encoder's value is 0. Should be close to 0.
     */
    @OutputUnit(UnitTypes.DEGREES)
    private static final double phi_0 = 0.0; // 6.35
    /**
     * The length from the shooter shaft to the shooter exit.
     */
    @OutputUnit(UnitTypes.METERS)
    private static final double l = 0.2; // 0.3

    public static double getAngleOfElevation1(double distanceFromCamera) {
        for (double outputAngle = 0.0; outputAngle <= 90.0; outputAngle += 0.5) {
            double exitX = distanceFromCamera - x_sl + x_s - l * Math.cos(Units.degreesToRadians(outputAngle + phi_0));
            double exitY = y_s + l * Math.sin(Units.degreesToRadians(outputAngle + phi_0));
            double y = q(0.0, Units.degreesToRadians(outputAngle + phi_0), exitX, exitY);
            if (y - 1.98 > 0) {
                return outputAngle;
    private final DormandPrince853Integrator integrator = new DormandPrince853Integrator(
            1.0e-8, 100.0, 1.0e-10, 1.0e-10);
    private final MixedAirDragODE ode = new MixedAirDragODE(TrajectoryConstants.a, TrajectoryConstants.b);

    public double getAngleOfElevation12(double distanceFromCamera) {
        double prevError = Integer.MAX_VALUE;

        for (double angle = 0.0; angle < 90.0; angle += 0.5) {
            double error = this.getTargetError(angle, distanceFromCamera);

            if (error > prevError) {
                return angle - 0.5;
            }
        }

        return 90.0;
    }

    private double getTargetError(double encoderAngle, double distanceFromCamera) {
        double degree = Units.degreesToRadians(encoderAngle + TrajectoryConstants.phi_0);
        double exitX = distanceFromCamera - TrajectoryConstants.x_sl + TrajectoryConstants.x_s;
        exitX -= TrajectoryConstants.l * FastMath.cos(degree);
        double exitY = TrajectoryConstants.y_s + TrajectoryConstants.l * FastMath.sin(degree);
        double[] y0 = {exitY, -TrajectoryConstants.v_0 * FastMath.cos(degree),
                TrajectoryConstants.v_0 * FastMath.sin(degree)};

        try {
            this.integrator.integrate(this.ode, exitX, y0, 0.2286, y0);
            return Math.abs(y0[0] - 2.0431125);
        } catch (NumberIsTooSmallException e) {
            return Integer.MAX_VALUE;
        }
    }
}
