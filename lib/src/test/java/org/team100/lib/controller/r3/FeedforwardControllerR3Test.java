package org.team100.lib.controller.r3;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.state.Control100;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.Model100;
import org.team100.lib.state.ModelR3;

public class FeedforwardControllerR3Test {
    private static final double DELTA = 0.001;

    @Test
    void testMotionless() {
        FeedforwardControllerR3 c = new FeedforwardControllerR3(0.01, 0.01, 0.01);
        assertFalse(c.atReference());
        GlobalVelocityR3 v = c.calculate(
                new ModelR3(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ModelR3(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ControlR3(
                        new Control100(0, 0),
                        new Control100(0, 0),
                        new Control100(0, 0)));
        assertEquals(0, v.x(), DELTA);
        assertEquals(0, v.y(), DELTA);
        assertEquals(0, v.theta(), DELTA);
        assertTrue(c.atReference());
    }

    @Test
    void testNotAtReference() {
        FeedforwardControllerR3 c = new FeedforwardControllerR3(0.01, 0.01, 0.01);
        assertFalse(c.atReference());
        GlobalVelocityR3 v = c.calculate(
                new ModelR3(
                        new Model100(1, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ModelR3(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ControlR3(
                        new Control100(0, 0),
                        new Control100(0, 0),
                        new Control100(0, 0)));
        assertEquals(0, v.x(), DELTA);
        assertEquals(0, v.y(), DELTA);
        assertEquals(0, v.theta(), DELTA);
        assertFalse(c.atReference());
    }

    @Test
    void testFeedforward() {
        FeedforwardControllerR3 c = new FeedforwardControllerR3(0.01, 0.01, 0.01);
        assertFalse(c.atReference());
        GlobalVelocityR3 v = c.calculate(
                new ModelR3(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ModelR3(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ControlR3(
                        new Control100(0, 1),
                        new Control100(0, 0),
                        new Control100(0, 0)));
        assertEquals(1, v.x(), DELTA);
        assertEquals(0, v.y(), DELTA);
        assertEquals(0, v.theta(), DELTA);
        assertTrue(c.atReference());
    }
}
