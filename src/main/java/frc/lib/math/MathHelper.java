package frc.lib.math;

import org.apache.commons.math3.util.FastMath;

public final class MathHelper {
    public static double getMean(double a, double b) {
        return (a + b) / 2.0;
    }

    public static boolean isWithinRange(double a, double b, double range) {
        return FastMath.abs(a - b) < range;
    }

    public static double getSign(double a) {
        return (a == 0) ? 0 : (a / FastMath.abs(a));
    }

    public static double applyMax(double a, double max) {
        return FastMath.min(max, FastMath.abs(a)) * getSign(a);
    }
}
