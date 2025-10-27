package org.team100.lib.commands.swerve.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.state.ModelR3;

class ManualFieldRelativeSpeedsTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testTwistZero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        ManualFieldRelativeSpeeds manual = new ManualFieldRelativeSpeeds(logger, limits);
        Velocity input = new Velocity(0, 0, 0);
        ModelR3 s = new ModelR3();
        GlobalVelocityR3 twist = manual.apply(s, input);
        assertEquals(0, twist.x(), DELTA);
        assertEquals(0, twist.y(), DELTA);
        assertEquals(0, twist.theta(), DELTA);
    }

    @Test
    void testTwistNonzero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        ManualFieldRelativeSpeeds manual = new ManualFieldRelativeSpeeds(logger, limits);
        // these inputs are clipped
        Velocity input = new Velocity(1, 2, 3);
        ModelR3 s = new ModelR3();
        GlobalVelocityR3 twist = manual.apply(s, input);
        assertEquals(0.447, twist.x(), DELTA);
        assertEquals(0.894, twist.y(), DELTA);
        assertEquals(2.828, twist.theta(), DELTA);
    }

}
