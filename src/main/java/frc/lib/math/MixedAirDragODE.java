package frc.lib.math;

import frc.lib.helpers.OutputUnit;
import frc.lib.helpers.UnitTypes;
import org.apache.commons.math4.core.jdkmath.JdkMath;
import org.apache.commons.math4.legacy.exception.DimensionMismatchException;
import org.apache.commons.math4.legacy.exception.MaxCountExceededException;
import org.apache.commons.math4.legacy.ode.FirstOrderDifferentialEquations;

/**
 * Defining the Ordinary Differential Equation system for considering both the first order and
 * the second order air drag to the note. Integrated by
 * {@link org.apache.commons.math4.legacy.ode.nonstiff.DormandPrince853Integrator}
 *
 * @see TrajectoryEstimator
 */
public class MixedAirDragODE implements FirstOrderDifferentialEquations {
    /**
     * The gravitational acceleration constant. Fixed.
     */
    @OutputUnit(UnitTypes.METERS_PER_SECOND_SQUARED)
    private static final double g = TrajectoryConstants.g;
    /**
     * The second order air drag constant of the note.
     * Can be set to zero. In this case, the second order air drag is ignored.
     */
    @OutputUnit(UnitTypes.ONE_OVER_METER)
    private final double a;
    /**
     * The first order air drag constant of the note.
     * Can be set to zero. In this case, the first order air drag is ignored.
     */
    @OutputUnit(UnitTypes.ONE_OVER_SECOND)
    private final double b;

    protected MixedAirDragODE(double a, double b) {
        this.a = a;
        this.b = b;
        assert a >= 0 && b >= 0;
    }

    /**
     * @return the total count of all unknown function y(t)s.
     */
    @Override
    public int getDimension() {
        return 3;
    }

    /**
     * Construct the ODE system with the following equations, let: <br>
     * <ul>
     *     <li>"t" = x = the x coordinate (horizontal) of the note from the origin (0, 0)</li>
     *     <li>"y[0]" = y = the y coordinate (vertical) of the note from the origin (0, 0)</li>
     *     <li>"y[1]" = dx/dt = Vx = the x velocity (horizontal) of the note</li>
     *     <li>"y[2]" = dy/dt = Vy = the y velocity (vertical) of the note</li>
     * </ul>
     * Where the origin (0, 0) is the intersection of the speaker wall and the carpet. <br>
     * Then the equations become: <br>
     * <ul>
     *     <li>dy/dx = Vy / Vx</li>
     *     <li>dVx/dx = -a * sqrt(Vx^2 + Vy^2) - b</li>
     *     <li>dVy/dx = (-g - a * Vy * sqrt(Vx^2 + Vy^2) - b * Vy) / Vx</li>
     * </ul>
     *
     * @param t the current x (horizontal) value with respect to the origin (0, 0).
     * @param y array containing the current value of y, Vx, and Vy.
     * @param yDot placeholder array where to put the derivatives of y, Vx, Vy with respect to x.
     */
    @Override
    public void computeDerivatives(double t, double[] y, double[] yDot) throws MaxCountExceededException, DimensionMismatchException {
        double d = JdkMath.sqrt(y[1] * y[1] + y[2] * y[2]);
        yDot[0] = y[2] / y[1];
        yDot[1] = -this.a * d - this.b;
        yDot[2] = (-g - this.a * y[2] * d - this.b * y[2]) / y[1];
    }
}
