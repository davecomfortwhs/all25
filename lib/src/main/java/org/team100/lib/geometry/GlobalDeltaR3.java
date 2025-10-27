package org.team100.lib.geometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * This is just a container for the difference between two poses, treating the
 * dimensions of each pose as independent. That is, it is a difference in R3,
 * not SE(2).
 * 
 * The SE(2) difference represents a *geodesic* in SE(2), and for differences
 * that include rotation, this will appear as a curved path -- usually not what is
 * desired.
 * 
 * The R3 difference represents a straight line in every axis.
 * 
 * This is useful for control problems where the dimensions are treated as
 * independent, e.g. if you have three separate proportional feedback
 * controllers, or if you want to interpolate the axes separately.
 */
public class GlobalDeltaR3 {
    private final Translation2d m_translation;
    private final Rotation2d m_rotation;

    public GlobalDeltaR3(Translation2d translation, Rotation2d rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }

    /** Return a delta from start to end. */
    public static GlobalDeltaR3 delta(Pose2d start, Pose2d end) {
        Translation2d t = end.getTranslation().minus(start.getTranslation());
        Rotation2d r = end.getRotation().minus(start.getRotation());
        return new GlobalDeltaR3(t, r);
    }

    public GlobalDeltaR3 limit(double cartesian, double rotation) {
        return new GlobalDeltaR3(
                new Translation2d(
                        MathUtil.clamp(m_translation.getX(), -cartesian, cartesian),
                        MathUtil.clamp(m_translation.getY(), -cartesian, cartesian)),
                new Rotation2d(
                        MathUtil.clamp(m_rotation.getRadians(), -rotation, rotation)));
    }

    public GlobalDeltaR3 times(double scalar) {
        return new GlobalDeltaR3(m_translation.times(scalar), m_rotation.times(scalar));
    }

    public GlobalDeltaR3 div(double scalar) {
        return new GlobalDeltaR3(m_translation.div(scalar), m_rotation.div(scalar));
    }

    public double getX() {
        return m_translation.getX();
    }

    public double getY() {
        return m_translation.getY();
    }

    public Translation2d getTranslation() {
        return m_translation;
    }

    public Rotation2d getRotation() {
        return m_rotation;
    }

    public double getRadians() {
        return m_rotation.getRadians();
    }

}
