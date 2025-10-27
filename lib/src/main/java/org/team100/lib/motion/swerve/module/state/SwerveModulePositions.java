package org.team100.lib.motion.swerve.module.state;

/**
 * Container for swerve module positions.
 * 
 * This is intended to avoid passing around an array of positions,
 * and having to remember which location corresponds to which index.
 */
public record SwerveModulePositions(
        SwerveModulePosition100 frontLeft,
        SwerveModulePosition100 frontRight,
        SwerveModulePosition100 rearLeft,
        SwerveModulePosition100 rearRight) {
    /** For when you don't care about which is which. */
    public SwerveModulePosition100[] all() {
        return new SwerveModulePosition100[] {
                frontLeft,
                frontRight,
                rearLeft,
                rearRight
        };
    }

    public SwerveModulePositions(SwerveModulePositions other) {
        this(other.frontLeft.copy(),
                other.frontRight.copy(),
                other.rearLeft.copy(),
                other.rearRight.copy());
    }

    public static SwerveModulePositions modulePositionFromDelta(
            SwerveModulePositions initial,
            SwerveModuleDeltas delta) {
        return new SwerveModulePositions(
                initial.frontLeft().plus(delta.frontLeft()),
                initial.frontRight().plus(delta.frontRight()),
                initial.rearLeft().plus(delta.rearLeft()),
                initial.rearRight().plus(delta.rearRight()));
    }

    public static SwerveModulePositions kZero() {
        return new SwerveModulePositions(
                new SwerveModulePosition100(),
                new SwerveModulePosition100(),
                new SwerveModulePosition100(),
                new SwerveModulePosition100());
    }

}
