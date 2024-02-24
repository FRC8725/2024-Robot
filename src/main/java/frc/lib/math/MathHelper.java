package frc.lib.math;

import org.apache.commons.math3.util.FastMath;

public final class MathHelper {
    public static double getMean(double a, double b) {
        return (a + b) / 2.0;
    }

    public static boolean isWithinRange(double a, double b, double range) {
        return FastMath.abs(a - b) < range;
    }
}
