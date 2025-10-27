package org.team100.lib.geometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * This is acceleration in the global reference frame, the second derivative of
 * Pose2d. This means "field relative" for robot navigation and control. It is
 * also useful for purposes other than navigation, e.g. planar mechanism
 * kinematics.
 * 
 * Units are meters, radians, and seconds.
 * 
 * This implements acceleration in R3, not SE(2); see README.md for details.
 */
public record GlobalAccelerationR3(double x, double y, double theta) {
    public GlobalVelocityR3 integrate(double dtSec) {
        return new GlobalVelocityR3(x * dtSec, y * dtSec, theta * dtSec);
    }

    public GlobalAccelerationR3 plus(GlobalAccelerationR3 other) {
        return new GlobalAccelerationR3(x + other.x, y + other.y, theta + other.theta);
    }

    public GlobalAccelerationR3 minus(GlobalAccelerationR3 other) {
        return new GlobalAccelerationR3(x - other.x, y - other.y, theta - other.theta);
    }

    public GlobalJerkR3 jerk(GlobalAccelerationR3 previous, double dt) {
        GlobalAccelerationR3 v = minus(previous).div(dt);
        return new GlobalJerkR3(v.x(), v.y(), v.theta());
    }

    public GlobalAccelerationR3 times(double scalar) {
        return new GlobalAccelerationR3(x * scalar, y * scalar, theta * scalar);
    }

    public GlobalAccelerationR3 div(double scalar) {
        return new GlobalAccelerationR3(x / scalar, y / scalar, theta / scalar);
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public static GlobalAccelerationR3 diff(
            GlobalVelocityR3 v1,
            GlobalVelocityR3 v2,
            double dtSec) {
        GlobalVelocityR3 dv = v2.minus(v1);
        return new GlobalAccelerationR3(dv.x() / dtSec, dv.y() / dtSec, dv.theta() / dtSec);
    }

    public GlobalAccelerationR3 clamp(double maxAccel, double maxAlpha) {
        double norm = Math.hypot(x, y);
        double ratio = 1.0;
        if (norm > 1e-3 && norm > maxAccel) {
            ratio = maxAccel / norm;
        }
        return new GlobalAccelerationR3(ratio * x, ratio * y, MathUtil.clamp(theta, -maxAlpha, maxAlpha));
    }

    public static GlobalAccelerationR3 fromVector(Matrix<N3, N1> v) {
        return new GlobalAccelerationR3(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(x, y, theta);
    }

    @Override
    public String toString() {
        return String.format("(%5.2f, %5.2f, %5.2f)", x, y, theta);
    }
}
