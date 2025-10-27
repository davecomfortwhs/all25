package org.team100.lib.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Various math utilities.
 */
public class Math100 {
    public static final boolean DEBUG = false;
    private static final double EPSILON = 1e-6;

    /**
     * Returns the real solutions to the quadratic ax^2 + bx + c.
     */
    public static List<Double> solveQuadratic(double a, double b, double c) {
        double disc = b * b - 4 * a * c;

        if (epsilonEquals(disc, 0.0)) {
            return List.of(-b / (2 * a));
        } else if (disc > 0.0) {
            return List.of(
                    (-b + Math.sqrt(disc)) / (2 * a),
                    (-b - Math.sqrt(disc)) / (2 * a));
        } else {
            return new ArrayList<>();
        }
    }

    public static boolean epsilonEquals(double x, double y) {
        return epsilonEquals(x, y, EPSILON);
    }

    public static boolean epsilonEquals(double x, double y, double epsilon) {
        return Math.abs(x - y) < epsilon;
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static double interpolate(double a, double b, double x) {
        if (x == 0)
            return a;
        if (x == 1)
            return b;
        x = limit(x, 0.0, 1.0);
        if (x < 1e-12)
            return a;
        if (x > (1 - 1e-12))
            return b;
        return a + (b - a) * x;
    }

    private Math100() {
    }

    /**
     * Produce an Euler angle equivalent to x but closer to measurement; might be
     * outside [-pi,pi].
     */
    public static double getMinDistance(double measurement, double x) {
        return MathUtil.angleModulus(x - measurement) + measurement;
    }

    public static Rotation2d getMinDistance(double measurement, Rotation2d x) {
        return new Rotation2d(MathUtil.angleModulus(x.getRadians() - measurement) + measurement);
    }

    public static double notNaN(double x) {
        if (Double.isNaN(x))
            throw new IllegalArgumentException("arg is NaN");
        return x;
    }

    /** Throw if x is out of range. This is a more strict version of "clamp" :-) */
    public static double throwIfOutOfRange(double x, double minX, double maxX) {
        if (x < minX)
            throw new IllegalArgumentException(String.format("arg is %f which is below %f", x, minX));
        if (x > maxX)
            throw new IllegalArgumentException(String.format("arg is %f which is above %f", x, maxX));
        return x;
    }

}
