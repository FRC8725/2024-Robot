package frc.lib.math;

import edu.wpi.first.math.util.Units;
import org.apache.commons.math4.core.jdkmath.JdkMath;
import org.apache.commons.math4.legacy.exception.NumberIsTooSmallException;
import org.apache.commons.math4.legacy.ode.nonstiff.DormandPrince853Integrator;

public class TrajectoryEstimator {
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

            prevError = error;
        }

        return 90.0;
    }

    private double getTargetError(double encoderAngle, double distanceFromCamera) {
        double degree = Units.degreesToRadians(encoderAngle + TrajectoryConstants.phi_0);
        double exitX = distanceFromCamera - TrajectoryConstants.x_sl + TrajectoryConstants.x_s;
        exitX -= TrajectoryConstants.l * JdkMath.cos(degree);
        double exitY = TrajectoryConstants.y_s + TrajectoryConstants.l * JdkMath.sin(degree);
        double[] y0 = {exitY, -TrajectoryConstants.v_0 * JdkMath.cos(degree),
                TrajectoryConstants.v_0 * JdkMath.sin(degree)};

        try {
            this.integrator.integrate(this.ode, exitX, y0, 0.2286, y0);
            return Math.abs(y0[0] - 2.0431125);
        } catch (NumberIsTooSmallException e) {
            return Integer.MAX_VALUE;
        }
    }
}
