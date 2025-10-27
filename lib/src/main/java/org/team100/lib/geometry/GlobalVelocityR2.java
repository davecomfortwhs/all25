package org.team100.lib.geometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Velocity in R2, companion to Translation2d. */
public record GlobalVelocityR2(double x, double y) {
    public static GlobalVelocityR2 fromPolar(Rotation2d angle, double speed) {
        return new GlobalVelocityR2(speed * angle.getCos(), speed * angle.getSin());
    }

    /** Pick up the translation component of v. */
    public static GlobalVelocityR2 fromSe2(GlobalVelocityR3 v) {
        return new GlobalVelocityR2(v.x(), v.y());
    }

    public GlobalVelocityR2 plus(GlobalVelocityR2 other) {
        return new GlobalVelocityR2(x + other.x, y + other.y);
    }

    public Translation2d integrate(Translation2d start, double dt) {
        return new Translation2d(start.getX() + x * dt, start.getY() + y * dt);
    }
}
