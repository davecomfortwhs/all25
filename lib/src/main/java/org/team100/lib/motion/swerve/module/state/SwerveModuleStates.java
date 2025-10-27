package org.team100.lib.motion.swerve.module.state;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Container for swerve module states.
 * 
 * This is intended to avoid passing around an array of states,
 * and having to remember which location corresponds to which index.
 */
public record SwerveModuleStates(
        SwerveModuleState100 frontLeft,
        SwerveModuleState100 frontRight,
        SwerveModuleState100 rearLeft,
        SwerveModuleState100 rearRight) {

    private static final Optional<Rotation2d> AHEAD = Optional.of(Rotation2d.kZero);
    private static final double STOP = 0.0;
    private static final double SLOW = 1.0;

    public static final SwerveModuleStates aheadSlow = new SwerveModuleStates(
            new SwerveModuleState100(SLOW, AHEAD),
            new SwerveModuleState100(SLOW, AHEAD),
            new SwerveModuleState100(SLOW, AHEAD),
            new SwerveModuleState100(SLOW, AHEAD));

    public static final SwerveModuleStates states0 = new SwerveModuleStates(
            new SwerveModuleState100(STOP, AHEAD),
            new SwerveModuleState100(STOP, AHEAD),
            new SwerveModuleState100(STOP, AHEAD),
            new SwerveModuleState100(STOP, AHEAD));

    public static final SwerveModuleStates states90 = new SwerveModuleStates(
            new SwerveModuleState100(STOP, Optional.of(new Rotation2d(Math.PI / 2))),
            new SwerveModuleState100(STOP, Optional.of(new Rotation2d(Math.PI / 2))),
            new SwerveModuleState100(STOP, Optional.of(new Rotation2d(Math.PI / 2))),
            new SwerveModuleState100(STOP, Optional.of(new Rotation2d(Math.PI / 2))));
    public static final SwerveModuleStates statesX = new SwerveModuleStates(
            // note range is [-pi,pi]
            new SwerveModuleState100(STOP, Optional.of(new Rotation2d(Math.PI / 4))),
            new SwerveModuleState100(STOP, Optional.of(new Rotation2d(-1 * Math.PI / 4))),
            new SwerveModuleState100(STOP, Optional.of(new Rotation2d(3 * Math.PI / 4))),
            new SwerveModuleState100(STOP, Optional.of(new Rotation2d(-3 * Math.PI / 4))));

    /** For when you don't care about which is which. */
    public SwerveModuleState100[] all() {
        return new SwerveModuleState100[] {
                frontLeft,
                frontRight,
                rearLeft,
                rearRight
        };
    }
}
