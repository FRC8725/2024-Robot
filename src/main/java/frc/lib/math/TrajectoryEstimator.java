package frc.lib.math;

import edu.wpi.first.math.geometry.Translation2d;
import org.apache.commons.math3.util.FastMath;

public class TrajectoryEstimator {
    public static double getAngleOfElevation(double distanceFromEdge) {
        if (distanceFromEdge >= 3.5) {
            return 25.51;
        }

        double a = -(3.096 + 1.0 / 3.0) * FastMath.pow(distanceFromEdge, 5);
        double b = 47.1 * FastMath.pow(distanceFromEdge, 4);
        double c = -(251.2 + 1.0 / 3.0) * FastMath.pow(distanceFromEdge, 3);
        double d = 656.325 * FastMath.pow(distanceFromEdge, 2);
        double e = -(853.159 + 1.0 / 3.0) * distanceFromEdge;
        double f = 490.45;
        return a + b + c + d + e + f;
    }
}
