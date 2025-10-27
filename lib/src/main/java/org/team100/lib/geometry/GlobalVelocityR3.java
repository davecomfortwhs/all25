package org.team100.lib.geometry;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * This is velocity in the global reference frame, the first derivative of
 * Pose2d. This means "field relative" for robot navigation and control. It is
 * similar to WPI ChassisSpeeds, but field-relative. It is also useful for
 * purposes other than navigation, e.g. planar mechanism kinematics.
 * 
 * Units are meters, radians, and seconds.
 * 
 * This implements velocity in R3, not SE(2); see README.md for details.
 */
public record GlobalVelocityR3(double x, double y, double theta) {

    public static final GlobalVelocityR3 ZERO = new GlobalVelocityR3(0, 0, 0);

    public static GlobalVelocityR3 zero() {
        return ZERO;
    }

    public static GlobalVelocityR3 velocity(Pose2d start, Pose2d end, double dt) {
        GlobalDeltaR3 d = GlobalDeltaR3.delta(start, end);
        return new GlobalVelocityR3(d.getX(), d.getY(), d.getRotation().getRadians()).div(dt);
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public GlobalVelocityR3 normalize() {
        double norm = norm();
        if (norm < 1e-6)
            return zero();
        return new GlobalVelocityR3(x, y, theta).times(1.0 / norm);
    }

    /** Field-relative course, or empty if slower than 1 micron/sec. */
    public Optional<Rotation2d> angle() {
        if (Math.abs(x) < 1e-6 && Math.abs(y) < 1e-6)
            return Optional.empty();
        return Optional.of(new Rotation2d(x, y));
    }

    public GlobalVelocityR3 plus(GlobalVelocityR3 other) {
        return new GlobalVelocityR3(x + other.x, y + other.y, theta + other.theta);
    }

    /** The return type here isn't really right. */
    public GlobalVelocityR3 minus(GlobalVelocityR3 other) {
        return new GlobalVelocityR3(x - other.x, y - other.y, theta - other.theta);
    }

    public GlobalAccelerationR3 accel(GlobalVelocityR3 previous, double dt) {
        GlobalVelocityR3 v = minus(previous).div(dt);
        return new GlobalAccelerationR3(v.x(), v.y(), v.theta());
    }

    public GlobalVelocityR3 times(double scalar) {
        return new GlobalVelocityR3(x * scalar, y * scalar, theta * scalar);
    }

    public GlobalVelocityR3 div(double scalar) {
        return new GlobalVelocityR3(x / scalar, y / scalar, theta / scalar);
    }

    public GlobalVelocityR3 times(double cartesian, double angular) {
        return new GlobalVelocityR3(x * cartesian, y * cartesian, theta * angular);
    }

    /** Dot product of translational part. */
    public double dot(GlobalVelocityR3 other) {
        return x * other.x + y * other.y;
    }

    /** Stopping distance at the specified acceleration. */
    public Translation2d stopping(double accel) {
        double speed = norm();
        double decelTime = speed / accel;
        // the unit here is wrong
        GlobalVelocityR3 dx = normalize().times(0.5 * speed * decelTime);
        return new Translation2d(dx.x, dx.y);
    }

    public GlobalVelocityR3 clamp(double maxVelocity, double maxOmega) {
        double norm = Math.hypot(x, y);
        double ratio = 1.0;
        if (norm > 1e-3 && norm > maxVelocity) {
            ratio = maxVelocity / norm;
        }
        return new GlobalVelocityR3(ratio * x, ratio * y, MathUtil.clamp(theta, -maxOmega, maxOmega));
    }

    @Override
    public String toString() {
        return String.format("(%5.2f, %5.2f, %5.2f)", x, y, theta);
    }

    public static GlobalVelocityR3 fromVector(Vector<N3> v) {
        return new GlobalVelocityR3(v.get(0), v.get(1), v.get(2));
    }

    public static GlobalVelocityR3 fromVector(Matrix<N3, N1> v) {
        return new GlobalVelocityR3(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(x, y, theta);
    }
}