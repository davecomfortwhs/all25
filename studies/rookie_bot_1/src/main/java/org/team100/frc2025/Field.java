package org.team100.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Joel eyeballed these on 10/23/25.
 * 
 * TODO: more field points, like scoring, pickup, etc.
 */
public class Field {
    public static final Pose2d START_RED_RIGHT = new Pose2d(0.55, 0.55, Rotation2d.kZero);
    public static final Pose2d START_RED_CENTER = new Pose2d(0.55, 1.10, Rotation2d.kZero);
    public static final Pose2d START_RED_LEFT = new Pose2d(0.55, 2.05, Rotation2d.kZero);
    public static final Pose2d START_BLUE_RIGHT = new Pose2d(0.55, 2.20, Rotation2d.kZero);
    public static final Pose2d START_BLUE_CENTER = new Pose2d(0.55, 2.97, Rotation2d.kZero);
    public static final Pose2d START_BLUE_LEFT = new Pose2d(0.55, 3.74, Rotation2d.kZero);
}
