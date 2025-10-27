package org.team100.lib.motion.prr;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.prr.CoordinatedKinematics.Joints;

import edu.wpi.first.math.geometry.Translation2d;

public class CoordinatedKinematicsTest {
    private static final double DELTA = 0.001;

    @Test
    void testForward() {
        CoordinatedKinematics k = new CoordinatedKinematics(1);
        // at the origin, arm points 1m up
        Translation2d f = k.forward(new Joints(0, 0));
        assertEquals(1, f.getX(), DELTA);
        assertEquals(0, f.getY(), DELTA);
        // raise 1m, so arm is 2m up
        f = k.forward(new Joints(1, 0));
        assertEquals(2, f.getX(), DELTA);
        assertEquals(0, f.getY(), DELTA);
        // straight out
        f = k.forward(new Joints(0, Math.PI / 2));
        assertEquals(0, f.getX(), DELTA);
        assertEquals(1, f.getY(), DELTA);
        // 45 degrees
        f = k.forward(new Joints(0, Math.PI / 4));
        assertEquals(0.707, f.getX(), DELTA);
        assertEquals(0.707, f.getY(), DELTA);
        // 45 degrees
        f = k.forward(new Joints(1, Math.PI / 4));
        assertEquals(1.707, f.getX(), DELTA);
        assertEquals(0.707, f.getY(), DELTA);
    }

    @Test
    void testInverse() {
        CoordinatedKinematics k = new CoordinatedKinematics(1);
        // at the origin, arm points 1m up
        Joints j = k.inverse(new Translation2d(1, 0));
        assertEquals(0, j.height(), DELTA);
        assertEquals(0, j.angle(), DELTA);
        // raise 1m, arm points 2m up
        j = k.inverse(new Translation2d(2, 0));
        assertEquals(1, j.height(), DELTA);
        assertEquals(0, j.angle(), DELTA);
        // straight out
        j = k.inverse(new Translation2d(0, 1));
        assertEquals(0, j.height(), DELTA);
        assertEquals(Math.PI / 2, j.angle(), DELTA);
        // 45 degrees?
        j = k.inverse(new Translation2d(0.707107, 0.707107));
        assertEquals(0, j.height(), DELTA);
        assertEquals(Math.PI / 4, j.angle(), DELTA);
        // 45 degrees?
        j = k.inverse(new Translation2d(1.707107, 0.707107));
        assertEquals(1, j.height(), DELTA);
        assertEquals(Math.PI / 4, j.angle(), DELTA);
        // these are unreachable
        assertNull(k.inverse(new Translation2d(0, 2))) ;
        assertNull(k.inverse(new Translation2d(0, -2)));
    }

}
