package org.team100.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Pose2dWithMotion.MotionDirection;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.trajectory.timing.TimingConstraint.MinMaxAcceleration;

import edu.wpi.first.math.geometry.Pose2d;

class SwerveDriveDynamicsConstraintTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testVelocity() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        SwerveDriveDynamicsConstraint c = new SwerveDriveDynamicsConstraint(logger, kinodynamics, 1, 1);

        // motionless
        double m = c.getMaxVelocity(new Pose2dWithMotion(
                Pose2d.kZero, new Pose2dWithMotion.MotionDirection(0, 0, 0), 0, 0)).getValue();
        assertEquals(5, m, DELTA);

        // moving in +x, no curvature, no rotation
        m = c.getMaxVelocity(new Pose2dWithMotion(
                new Pose2d(),
                new MotionDirection(1, 0, 0),
                0, 0)).getValue();
        // max allowed velocity is full speed
        assertEquals(5, m, DELTA);

        // moving in +x, 5 rad/meter
        m = c.getMaxVelocity(new Pose2dWithMotion(
                new Pose2d(),
                new MotionDirection(1, 0, 5),
                0, 0)).getValue();
        // at 5 rad/m with 0.5m sides the fastest you can go is 1.55 m/s.
        assertEquals(1.925, m, DELTA);

        // max wheel speed 5 m/s
        // wheelsbase/track 0.5 m
        // so radius to center is 0.25 * sqrt(2) = 0.356
        // traveling 1 m/s, there are 4 m/s available for the fastest wheel
        // which means 11.314 rad/s, and also 11.314 rad/m since we're going 1 m/s.
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(),
                new MotionDirection(1, 0, 11.313708),
                0, 0);
        m = c.getMaxVelocity(
                state)
                .getValue();
        // verify corner velocity is full scale
        assertEquals(5, c.maxV());
        // this should be feasible; note it's not exactly 1 due to discretization
        assertEquals(1.036, m, DELTA);

    }

    @Test
    void testAccel() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        SwerveDriveDynamicsConstraint c = new SwerveDriveDynamicsConstraint(logger, kinodynamics, 1, 1);
        // this is constant
        MinMaxAcceleration m = c.getMinMaxAcceleration(new Pose2dWithMotion(
                Pose2d.kZero, new Pose2dWithMotion.MotionDirection(0, 0, 0), 0, 0), 0);
        assertEquals(-20, m.getMinAccel(), DELTA);
        assertEquals(10, m.getMaxAccel(), DELTA);
    }
}
