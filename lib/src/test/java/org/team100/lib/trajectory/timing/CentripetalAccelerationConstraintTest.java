package org.team100.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Pose2dWithMotion.MotionDirection;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;

import edu.wpi.first.math.geometry.Pose2d;

class CentripetalAccelerationConstraintTest {
    private static final double DELTA = 0.001;
    private static final double CENTRIPETAL_SCALE = 1.0;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSimple() {
        assertEquals(8.166, SwerveKinodynamicsFactory.forTest().getMaxCapsizeAccelM_S2(), DELTA);

        // 1 rad/m curve, 8 m/s^2 limit => 2.8 m/s
        CapsizeAccelerationConstraint c = new CapsizeAccelerationConstraint(
                logger,
                SwerveKinodynamicsFactory.forTest(),
                CENTRIPETAL_SCALE);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new MotionDirection(0, 0, 0), 1, 0);
        // motionless, so 100% of the capsize accel is available
        assertEquals(-8.166, c.getMinMaxAcceleration(p, 0).getMinAccel(), DELTA);
        assertEquals(8.166, c.getMinMaxAcceleration(p, 0).getMaxAccel(), DELTA);
        assertEquals(2.857, c.getMaxVelocity(p).getValue(), DELTA);
    }

    @Test
    void testSimpleMoving() {
        assertEquals(8.166, SwerveKinodynamicsFactory.forTest().getMaxCapsizeAccelM_S2(), DELTA);

        // 1 rad/m curve, 8 m/s^2 limit => 2.8 m/s
        CapsizeAccelerationConstraint c = new CapsizeAccelerationConstraint(
                logger,
                SwerveKinodynamicsFactory.forTest(),
                CENTRIPETAL_SCALE);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new MotionDirection(1, 0, 0), 1, 0);
        // moving, only some of the capsize accel is available
        assertEquals(-5.257, c.getMinMaxAcceleration(p, 2.5).getMinAccel(), DELTA);
        assertEquals(5.257, c.getMinMaxAcceleration(p, 2.5).getMaxAccel(), DELTA);
        assertEquals(2.857, c.getMaxVelocity(p).getValue(), DELTA);
    }

    @Test
    void testSimpleOverspeed() {
        assertEquals(8.166, SwerveKinodynamicsFactory.forTest().getMaxCapsizeAccelM_S2(), DELTA);

        // 1 rad/m curve, 8 m/s^2 limit => 2.8 m/s
        CapsizeAccelerationConstraint c = new CapsizeAccelerationConstraint(
                logger,
                SwerveKinodynamicsFactory.forTest(),
                CENTRIPETAL_SCALE);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new MotionDirection(1, 0, 0), 1, 0);
        // above the velocity limit
        assertEquals(-1, c.getMinMaxAcceleration(p, 3).getMinAccel(), DELTA);
        assertEquals(0, c.getMinMaxAcceleration(p, 3).getMaxAccel(), DELTA);
        assertEquals(2.857, c.getMaxVelocity(p).getValue(), DELTA);
    }

    @Test
    void testSimple2() {
        assertEquals(4.083, SwerveKinodynamicsFactory.forTest2().getMaxCapsizeAccelM_S2(), DELTA);
        // 1 rad/m curve, 4 m/s^2 limit => 2 m/s
        CapsizeAccelerationConstraint c = new CapsizeAccelerationConstraint(
                logger,
                SwerveKinodynamicsFactory.forTest2(),
                CENTRIPETAL_SCALE);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new MotionDirection(0, 0, 0), 1, 0);
        assertEquals(-4.083, c.getMinMaxAcceleration(p, 0).getMinAccel(), DELTA);
        assertEquals(4.083, c.getMinMaxAcceleration(p, 0).getMaxAccel(), DELTA);
        assertEquals(2.021, c.getMaxVelocity(p).getValue(), DELTA);
    }

    @Test
    void testStraightLine() {
        assertEquals(4.083, SwerveKinodynamicsFactory.forTest2().getMaxCapsizeAccelM_S2(), DELTA);
        // no curvature
        CapsizeAccelerationConstraint c = new CapsizeAccelerationConstraint(
                logger,
                SwerveKinodynamicsFactory.forTest2(),
                CENTRIPETAL_SCALE);
        Pose2dWithMotion p = new Pose2dWithMotion(
                new Pose2d(), new MotionDirection(1, 0, 0), 0, 0);
        assertEquals(-4.083, c.getMinMaxAcceleration(p, 0).getMinAccel(), DELTA);
        assertEquals(4.083, c.getMinMaxAcceleration(p, 0).getMaxAccel(), DELTA);
        assertEquals(Double.POSITIVE_INFINITY, c.getMaxVelocity(p).getValue(), DELTA);
    }

}
