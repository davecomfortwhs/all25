package org.team100.frc2025.parkinglot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.prr.Config;
import org.team100.lib.motion.prr.JointForce;

public class GravityTest {
    private static final double DELTA = 0.001;

    @Test
    void test0() {
        Gravity g = new Gravity(10, 1, 0.1, 1, 0.5, 1);
        // arm and hand both horizontal = maximum torque
        JointForce jf = g.get(new Config(1, Math.PI/2, 0));
        // total mass is 2kg, g is 10
        assertEquals(-20, jf.elevator(), DELTA);
        // 1kg * g * 1.1m + 1kg * g * 0.5
        assertEquals(16, jf.shoulder(), DELTA);
        // 1kg * g * 0.1m
        assertEquals(1, jf.wrist(), DELTA);
    }

    @Test
    void test1() {
        Gravity g = new Gravity(10, 1, 0.1, 1, 0.5, 1);
        // arm horizontal, hand facing up
        JointForce jf = g.get(new Config(1, Math.PI/2, -Math.PI/2));
        // total mass is 2kg, g is 10
        assertEquals(-20, jf.elevator(), DELTA);
        // 1kg * g * 0.5
        assertEquals(15, jf.shoulder(), DELTA);
        // no lever arm here
        assertEquals(0, jf.wrist(), DELTA);
    }

    @Test
    void test2() {
        Gravity g = new Gravity(10, 1, 0.1, 1, 0.5, 1);
        // arm up, hand out
        JointForce jf = g.get(new Config(1, 0, Math.PI/2));
        // total mass is 2kg, g is 10
        assertEquals(-20, jf.elevator(), DELTA);
        // 1kg * g * 0.1
        assertEquals(1, jf.shoulder(), DELTA);
        // 1kg * g * 0.1
        assertEquals(1, jf.wrist(), DELTA);
    }

    @Test
    void test3() {
        Gravity g = new Gravity(10, 1, 0.1, 1, 0.5, 1);
        // reaching backwards
        JointForce jf = g.get(new Config(1, -Math.PI/2, 0));
        // total mass is 2kg, g is 10
        assertEquals(-20, jf.elevator(), DELTA);
        // 1kg * g * 1.1m + 1kg * g * 0.5
        assertEquals(-16, jf.shoulder(), DELTA);
        // 1kg * g * 0.1m
        assertEquals(-1, jf.wrist(), DELTA);
    }

}
