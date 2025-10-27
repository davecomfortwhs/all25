package org.team100.lib.profile.roadrunner;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class TrapezoidProfileGeneratorTest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;

    @Test
    void testUnlimitedJerk() {
        MotionState start = new MotionState(0, 0, 0, 0);
        MotionProfile p = TrapezoidProfileGenerator.unlimitedJerk(start, 2, 6);
        // end-state accel is max (because the transition to cruise can be sharp in the
        // non-jerk-limited case.
        assertEquals(6.000, p.get(p.duration()).a(), DELTA);
        for (double t = 0; t < p.duration(); t += 0.01) {
            MotionState state = p.get(t);
            double x = state.x();
            double v = state.v();
            double a = state.a();
            double j = state.j();
            if (DEBUG)
                System.out.printf("%8.3f %8.3f %8.3f %8.3f %8.3f\n", t, x, v, a, j);
        }
    }
}
