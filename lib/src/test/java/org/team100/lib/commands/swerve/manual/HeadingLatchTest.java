package org.team100.lib.commands.swerve.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.hid.Velocity;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.geometry.Rotation2d;

class HeadingLatchTest {
    private static final double DELTA = 0.001;

    @Test
    void testInit() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        HeadingLatch l = new HeadingLatch();
        Model100 s = new Model100();
        Rotation2d pov = null;
        Velocity input = new Velocity(0, 0, 0);
        Rotation2d desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertNull(desiredRotation);
    }

    @Test
    void testLatch() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        HeadingLatch l = new HeadingLatch();
        Model100 s = new Model100();
        Rotation2d pov = Rotation2d.kCCW_Pi_2;
        Velocity input = new Velocity(0, 0, 0);
        Rotation2d desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), DELTA);
        pov = null;
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), DELTA);
    }

    @Test
    void testUnLatch() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        HeadingLatch l = new HeadingLatch();
        Model100 s = new Model100();
        Rotation2d pov = Rotation2d.kCCW_Pi_2;
        Velocity input = new Velocity(0, 0, 0);
        Rotation2d desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), DELTA);
        pov = null;
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), DELTA);
        input = new Velocity(0, 0, 1);
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertNull(desiredRotation);
    }

    @Test
    void testExplicitUnLatch() {
        Experiments.instance.testOverride(Experiment.StickyHeading, false);
        HeadingLatch l = new HeadingLatch();
        Model100 s = new Model100();
        Rotation2d pov = Rotation2d.kCCW_Pi_2;
        Velocity input = new Velocity(0, 0, 0);
        Rotation2d desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), DELTA);
        pov = null;
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(Math.PI / 2, desiredRotation.getRadians(), DELTA);
        l.unlatch();
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertNull(desiredRotation);
    }

    @Test
    void testSticky() {
        Experiments.instance.testOverride(Experiment.StickyHeading, true);
        HeadingLatch l = new HeadingLatch();
        Model100 s = new Model100(1, 1);
        Rotation2d pov = null;
        // driver steering, latch does nothing.
        Velocity input = new Velocity(0, 0, 1);
        Rotation2d desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertNull(desiredRotation);

        // let go of the steering stick, latch uses current
        s = new Model100(1, 1);
        input = new Velocity(0, 0, 0);
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        // max A = 10 rad/s^2
        // V = 1 rad/s
        // t = 0.1 sec
        // dx = 0.05 rad
        // setpoint = 1.05 rad
        assertEquals(1.05, desiredRotation.getRadians(), DELTA);

        // latch remembers even when current changes
        desiredRotation = l.latchedRotation(10, s, pov, input.theta());
        assertEquals(1.05, desiredRotation.getRadians(), DELTA);
    }
}
