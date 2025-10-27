package org.team100.lib.config;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Notice the difference between the naive feedforward and the more-correct
 * discretized one.
 */
class FeedforwardTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    @SuppressWarnings("removal")
    @Test
    void testWPI() {
        // current v = 1, want a = 1.
        // naive model says u = 2.
        assertEquals(2,
                new SimpleMotorFeedforward(0, 1, 1).calculate(1, 1),
                DELTA);

        // current v = 1, a = 1 for 1 sec => next v = 2.
        // discrete model knows that u = 2 applied for 1 sec won't yield
        // v = 2 because the back EMF will increase along the way.
        // if you want v = 2 after 1 sec, you need to push harder.
        assertEquals(2.582,
                new SimpleMotorFeedforward(0, 1, 1, 1).calculateWithVelocities(1, 2),
                DELTA);

        // for the usual time step this is a small correction.
        assertEquals(2.010,
                new SimpleMotorFeedforward(0, 1, 1).calculateWithVelocities(1, 1.02), DELTA);
    }

    @Test
    void test100() {
        // behaves the same as the naive model above, ignoring friction.
        Feedforward100 ff100 = new Feedforward100(log, 1, 1, 1, 0, 0, 0);
        assertEquals(1, ff100.velocityFFVolts(1), DELTA);
        assertEquals(1, ff100.accelFFVolts(1, 1), DELTA);
    }

    @Test
    void testkD() {
        // kd is lower
        Feedforward100 ff100 = new Feedforward100(log, 1, 1, 0.1, 0, 0, 0);
        assertEquals(1, ff100.accelFFVolts(1, 1), DELTA);
        assertEquals(0.1, ff100.accelFFVolts(-1, 1), DELTA);
    }

    /** I forgot an abs() in the friction term, so this verifies it. */
    @Test
    void testFriction() {
        // static friction = 2, dynamic friction = 1
        Feedforward100 ff100 = new Feedforward100(log, 1, 1, 1, 2, 1, 1);
        // under the static friction limit, so this is static
        assertEquals(2, ff100.frictionFFVolts(0.5), DELTA);
        // over the static friction limit, so sliding
        assertEquals(1, ff100.frictionFFVolts(2), DELTA);
        // under the static friction limit, so this is static
        assertEquals(-2, ff100.frictionFFVolts(-0.5), DELTA);
        // over the static friction limit, so sliding
        assertEquals(-1, ff100.frictionFFVolts(-2), DELTA);
        // want to go negative, get negative
        assertEquals(-2, ff100.frictionFFVolts(-0.5), DELTA);
        // moving positive, want to go negative, get negative
        assertEquals(-1, ff100.frictionFFVolts(-2), DELTA);
    }

}
