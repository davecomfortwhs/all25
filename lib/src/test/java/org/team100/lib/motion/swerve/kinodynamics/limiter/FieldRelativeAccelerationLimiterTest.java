package org.team100.lib.motion.swerve.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GlobalAccelerationR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;

public class FieldRelativeAccelerationLimiterTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testMotionless() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter limiter = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        GlobalVelocityR3 result = limiter.apply(
                new GlobalVelocityR3(0, 0, 0),
                new GlobalVelocityR3(0, 0, 0));
        assertEquals(0, result.x(), DELTA);
        assertEquals(0, result.y(), DELTA);
        assertEquals(0, result.theta(), DELTA);
    }

    @Test
    void testConstrained() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter limiter = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        GlobalVelocityR3 result = limiter.apply(
                new GlobalVelocityR3(0, 0, 0),
                new GlobalVelocityR3(1, 0, 0));
        assertEquals(0.02, result.x(), DELTA);
        assertEquals(0, result.y(), DELTA);
        assertEquals(0, result.theta(), DELTA);
    }

    @Test
    void testCartesianScale() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter limiter = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        GlobalVelocityR3 prev = new GlobalVelocityR3(0, 0, 0);
        GlobalVelocityR3 target = new GlobalVelocityR3(1, 0, 0);
        GlobalAccelerationR3 accel = target.accel(
                prev,
                TimedRobot100.LOOP_PERIOD_S);
        assertEquals(50, accel.x(), DELTA);
        double scale = limiter.cartesianScale(prev, target, accel);
        // allowed accel is 1, desired is 50, so scale is 0.02.
        assertEquals(0.02, scale, DELTA);
    }

    @Test
    void testAlpha() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter limiter = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        GlobalVelocityR3 result = limiter.apply(
                new GlobalVelocityR3(0, 0, 0),
                new GlobalVelocityR3(1, 0, 1));
        assertEquals(0.02, result.x(), DELTA);
        assertEquals(0, result.y(), DELTA);
        assertEquals(0.02, result.theta(), DELTA);
    }

    @Test
    void testAlphaRatio() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter limiter = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        GlobalVelocityR3 result = limiter.apply(
                new GlobalVelocityR3(0, 0, 0),
                new GlobalVelocityR3(1, 0, 10));
        assertEquals(0.017, result.x(), DELTA);
        assertEquals(0, result.y(), DELTA);
        assertEquals(0.170, result.theta(), DELTA);
    }

    @Test
    void testPureAlpha() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        FieldRelativeAccelerationLimiter limiter = new FieldRelativeAccelerationLimiter(logger, limits, 1, 1);
        GlobalVelocityR3 result = limiter.apply(
                new GlobalVelocityR3(0, 0, 0),
                new GlobalVelocityR3(0, 0, 1));
        assertEquals(0, result.x(), DELTA);
        assertEquals(0, result.y(), DELTA);
        assertEquals(0.170, result.theta(), DELTA);
    }
}
