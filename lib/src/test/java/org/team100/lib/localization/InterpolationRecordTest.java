package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.motion.swerve.module.state.SwerveModulePositions;
import org.team100.lib.state.Model100;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.math.geometry.Rotation2d;

class InterpolationRecordTest {
    // this is a 0.5 m square.
    SwerveKinodynamics l = SwerveKinodynamicsFactory.forTest();

    @Test
    void testInterp0() {
        // initially at rest, finally in motion.
        // what does the interpolator do?
        ModelR3 s0 = new ModelR3();
        SwerveModulePositions p0 = new SwerveModulePositions(
                new SwerveModulePosition100(0, Optional.empty()),
                new SwerveModulePosition100(0, Optional.empty()),
                new SwerveModulePosition100(0, Optional.empty()),
                new SwerveModulePosition100(0, Optional.empty()));
        InterpolationRecord r0 = new InterpolationRecord(
                l.getKinematics(), s0, p0);
        assertEquals(0, r0.m_state.theta().x(), 0.001);
        assertEquals(0, r0.m_state.theta().v(), 0.001);

        // 0.5m square means r = sqrt(2)/2; each wheel moves r, we turn 1 radian here.
        // if we're moving 1 rad/s now, and we moved 1 rad,
        // x = 1/2 a t ^ 2 = 1
        // v = a t = 1
        // so
        // 1 = 1/2 * 1 * t; t = 2, a = 0.5.

        ModelR3 s1 = new ModelR3(new Model100(), new Model100(), new Model100(1, 1));
        SwerveModulePositions p1 = new SwerveModulePositions(
                new SwerveModulePosition100(Math.sqrt(2) / 2, Optional.of(new Rotation2d(3 * Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 2, Optional.of(new Rotation2d(Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 2, Optional.of(new Rotation2d(-3 * Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 2, Optional.of(new Rotation2d(-Math.PI / 4))));
        // current speed is 1 rad/s
        InterpolationRecord r1 = new InterpolationRecord(
                l.getKinematics(), s1, p1);

        assertEquals(1, r1.m_state.theta().x(), 0.001);
        assertEquals(1, r1.m_state.theta().v(), 0.001);

        {
            // t=0 should return r0. note "t" is not time
            InterpolationRecord lerp = r0.interpolate(r1, 0.0);
            assertEquals(0, lerp.m_state.theta().x(), 0.001);
            assertEquals(0, lerp.m_state.theta().v(), 0.001);
        }
        {
            // t=1 should return r1. note this is a special case.
            InterpolationRecord lerp = r0.interpolate(r1, 1.0);
            assertEquals(1, lerp.m_state.theta().x(), 0.001);
            assertEquals(1, lerp.m_state.theta().v(), 0.001);
        }
        {
            // what about t=0.5? a is constant, so the angle goes
            // as t^2 ...
            // which is not what we do. this is just linear in all
            // dimensions which is not consistent but better than 0 or 1.
            InterpolationRecord lerp = r0.interpolate(r1, 0.5);
            // the beginning was unknown so the end is used for the whole time
            assertEquals(1.0, lerp.m_state.theta().x(), 0.001);
            assertEquals(0.5, lerp.m_state.theta().v(), 0.001);
        }
    }

    @Test
    void testInterp1() {
        ModelR3 s0 = new ModelR3();
        // initally driving straight
        SwerveModulePositions p0 = new SwerveModulePositions(
                new SwerveModulePosition100(0, Optional.of(new Rotation2d())),
                new SwerveModulePosition100(0, Optional.of(new Rotation2d())),
                new SwerveModulePosition100(0, Optional.of(new Rotation2d())),
                new SwerveModulePosition100(0, Optional.of(new Rotation2d())));
        InterpolationRecord r0 = new InterpolationRecord(
                l.getKinematics(), s0, p0);
        assertEquals(0, r0.m_state.theta().x(), 0.001);
        assertEquals(0, r0.m_state.theta().v(), 0.001);

        // 0.5m square means r = sqrt(2)/2; each wheel moves r, we turn 1 radian here.
        // if we're moving 1 rad/s now, and we moved 1 rad,
        // x = 1/2 a t ^ 2 = 1
        // v = a t = 1
        // so
        // 1 = 1/2 * 1 * t; t = 2, a = 0.5.

        ModelR3 s1 = new ModelR3(new Model100(), new Model100(), new Model100(1, 1));
        SwerveModulePositions p1 = new SwerveModulePositions(
                new SwerveModulePosition100(Math.sqrt(2) / 2, Optional.of(new Rotation2d(3 * Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 2, Optional.of(new Rotation2d(Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 2, Optional.of(new Rotation2d(-3 * Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 2, Optional.of(new Rotation2d(-Math.PI / 4))));
        // current speed is 1 rad/s
        InterpolationRecord r1 = new InterpolationRecord(
                l.getKinematics(), s1, p1);

        assertEquals(1, r1.m_state.theta().x(), 0.001);
        assertEquals(1, r1.m_state.theta().v(), 0.001);

        {
            // t=0 should return r0. note "t" is not time
            InterpolationRecord lerp = r0.interpolate(r1, 0.0);
            assertEquals(0, lerp.m_state.theta().x(), 0.001);
            assertEquals(0, lerp.m_state.theta().v(), 0.001);
        }
        {
            // t=1 should return r1. note this is a special case.
            InterpolationRecord lerp = r0.interpolate(r1, 1.0);
            assertEquals(1, lerp.m_state.theta().x(), 0.001);
            assertEquals(1, lerp.m_state.theta().v(), 0.001);
        }
        {
            InterpolationRecord lerp = r0.interpolate(r1, 0.5);
            // i dunno if this is right, but it's kinda in the ballpark
            assertEquals(0.653, lerp.m_state.theta().x(), 0.001);
            assertEquals(0.5, lerp.m_state.theta().v(), 0.001);
        }
    }

}
