package org.team100.lib.motion.swerve.module.state;

import java.util.Objects;
import java.util.Optional;

import org.team100.lib.motion.swerve.kinodynamics.struct.SwerveModulePosition100Struct;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.util.struct.StructSerializable;

/**
 * This is a copy of {@link edu.wpi.first.math.kinematics.SwerveModulePosition}
 * but with optional rotation, working around the incorrect behavior of
 * Rotation2d(0, 0).
 * 
 * Represents the state of one swerve module.
 * Uses the "unwrapped" angle; consumers should use MathUtil.angleModulus() if
 * they want the wrapped one.
 */
public class SwerveModulePosition100
        implements Comparable<SwerveModulePosition100>,
        Interpolatable<SwerveModulePosition100>,
        StructSerializable {
    private static final boolean DEBUG = false;
    /** Distance measured by the wheel of the module. */
    public double distanceMeters;

    /**
     * Angle of the module. It can be empty, in cases where the angle is
     * indeterminate (e.g. calculating the angle required for zero speed).
     * 
     * This is "unwrapped": its domain is infinite, not periodic within +/- pi.
     */
    public Optional<Rotation2d> unwrappedAngle = Optional.empty();

    /** SwerveModulePosition struct for serialization. */
    public static final SwerveModulePosition100Struct struct = new SwerveModulePosition100Struct();

    /** Zero distance and empty angle. */
    public SwerveModulePosition100() {
    }

    public SwerveModulePosition100(double distanceMeters, Optional<Rotation2d> unwrappedAngle) {
        this.distanceMeters = distanceMeters;
        this.unwrappedAngle = unwrappedAngle;
    }

    /**
     * Tests the *unwrapped* angle for equality, not the wrapped one which is the
     * default Rotation2d behavior.
     */
    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof SwerveModulePosition100 other))
            return false;

        if (Math.abs(other.distanceMeters - distanceMeters) > 1E-9)
            return false;
        if (unwrappedAngle.isEmpty() && other.unwrappedAngle.isPresent())
            return false;
        if (unwrappedAngle.isPresent() && other.unwrappedAngle.isEmpty())
            return false;
        if (unwrappedAngle.isEmpty() && other.unwrappedAngle.isEmpty())
            return true;
        if (Math.abs(unwrappedAngle.get().getRadians() - other.unwrappedAngle.get().getRadians()) > 1e-9)
            return false;
        return true;
    }

    @Override
    public int hashCode() {
        return Objects.hash(distanceMeters, unwrappedAngle);
    }

    /**
     * Compares two swerve module positions. One swerve module is "greater" than the
     * other if its
     * distance is higher than the other.
     *
     * @param other The other swerve module.
     * @return 1 if this is greater, 0 if both are equal, -1 if other is greater.
     */
    @Override
    public int compareTo(SwerveModulePosition100 other) {
        return Double.compare(this.distanceMeters, other.distanceMeters);
    }

    @Override
    public String toString() {
        return String.format(
                "SwerveModulePosition(Distance: %.2f m, Angle: %s)", distanceMeters, unwrappedAngle);
    }

    /**
     * Returns a copy of this swerve module position.
     *
     * @return A copy.
     */
    public SwerveModulePosition100 copy() {
        return new SwerveModulePosition100(distanceMeters, unwrappedAngle);
    }

    @Override
    public SwerveModulePosition100 interpolate(SwerveModulePosition100 endValue, double t) {
        double distLerp = MathUtil.interpolate(distanceMeters, endValue.distanceMeters, t);
        if (this.unwrappedAngle.isEmpty() && endValue.unwrappedAngle.isEmpty()) {
            // no angle information at all == no idea where we are, just return zero.
            return new SwerveModulePosition100(0.0, Optional.empty());
        }
        if (this.unwrappedAngle.isEmpty()) {
            // start is unknown but end is known, so use end.
            Rotation2d angleLerp = endValue.unwrappedAngle.get();
            return new SwerveModulePosition100(distLerp, Optional.of(angleLerp));
        }
        if (endValue.unwrappedAngle.isEmpty()) {
            // start is known but end is not, so use start.
            Rotation2d angleLerp = unwrappedAngle.get();
            return new SwerveModulePosition100(distLerp, Optional.of(angleLerp));
        }
        // both start and end are known, so interpolate.
        Rotation2d angleLerp = unwrappedAngle.get().interpolate(endValue.unwrappedAngle.get(), t);
        return new SwerveModulePosition100(distLerp, Optional.of(angleLerp));
    }

    /**
     * Given the start position, roll in the delta direction by the delta distance.
     * Note: this ignores the start direction, and so can produce strange results if
     * the difference between start and end angles is large.
     */
    public SwerveModulePosition100 plus(SwerveModuleDelta delta) {
        double posM = distanceMeters + delta.distanceMeters;
        if (delta.wrappedAngle.isPresent()) {
            if (DEBUG) {
                if (unwrappedAngle.isPresent()) {
                    // note wrapping here
                    Rotation2d angleDiff = delta.wrappedAngle.get().minus(unwrappedAngle.get());
                    if (Math.abs(angleDiff.getRadians()) > 0.2) {
                        System.out.printf("very fast steering start: %f end: %f\n",
                                unwrappedAngle.get().getRadians(), delta.wrappedAngle.get().getRadians());
                    }
                }
            }
            return new SwerveModulePosition100(posM, delta.wrappedAngle);
        }
        // if there's no delta angle, we're not going anywhere.
        if (DEBUG)
            System.out.println("no delta angle");
        return this;
    }

}
