package org.team100.lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * For constructing splines, paths, and trajectories.
 * 
 * @param translation
 * @param heading     where the front of the robot is facing
 * @param course      the direction the robot is moving
 */
public record HolonomicPose2d(
        Translation2d translation,
        Rotation2d heading,
        Rotation2d course) {

    public static HolonomicPose2d make(Pose2d p, Rotation2d course) {
        return new HolonomicPose2d(p.getTranslation(), p.getRotation(), course);
    }

    public static HolonomicPose2d make(Pose2d p, double course) {
        return new HolonomicPose2d(
                p.getTranslation(),
                p.getRotation(),
                new Rotation2d(course));
    }

    public static HolonomicPose2d make(double x, double y, double heading, double course) {
        return new HolonomicPose2d(
                new Translation2d(x, y), new Rotation2d(heading), new Rotation2d(course));
    }

    /** For tank drive, heading and course are the same. */
    public static HolonomicPose2d tank(double x, double y, double heading) {
        return new HolonomicPose2d(
                new Translation2d(x, y), new Rotation2d(heading), new Rotation2d(heading));
    }

    /**
     * For tank drive, heading and course are the same. This is like the WPI
     * trajectory.
     */
    public static HolonomicPose2d tank(Pose2d p) {
        return new HolonomicPose2d(
                p.getTranslation(), p.getRotation(), p.getRotation());
    }
}
