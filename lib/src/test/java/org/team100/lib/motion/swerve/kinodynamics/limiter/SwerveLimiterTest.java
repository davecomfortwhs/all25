package org.team100.lib.motion.swerve.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.profile.incremental.IncrementalProfile;
import org.team100.lib.profile.incremental.TrapezoidIncrementalProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

public class SwerveLimiterTest {
    private static final double DELTA = 0.001;
    private static final boolean DEBUG = false;
    private static final SwerveKinodynamics KINEMATIC_LIMITS = SwerveKinodynamicsFactory.limiting();
    private final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    /** The setpoint generator never changes the field-relative course. */
    @Test
    void courseInvariant() {
        GlobalVelocityR3 target = new GlobalVelocityR3(0, 0, 0);
        SwerveLimiter limiter = new SwerveLimiter(logger, KINEMATIC_LIMITS, () -> 12);

        {
            // motionless
            GlobalVelocityR3 prevSetpoint = new GlobalVelocityR3(0, 0, 0);
            limiter.updateSetpoint(prevSetpoint);
            GlobalVelocityR3 setpoint = limiter.apply(target);
            assertTrue(prevSetpoint.angle().isEmpty());
            assertTrue(setpoint.angle().isEmpty());
        }
        {
            // at max speed, 45 to the left and spinning
            GlobalVelocityR3 speed = new GlobalVelocityR3(2.640, 2.640, 3.733);
            GlobalVelocityR3 prevSetpoint = speed;
            limiter.updateSetpoint(prevSetpoint);
            GlobalVelocityR3 setpoint = limiter.apply(target);
            assertEquals(Math.PI / 4, prevSetpoint.angle().get().getRadians(), 1e-12);
            assertEquals(3.733, prevSetpoint.norm(), DELTA);
            assertEquals(3.733, prevSetpoint.theta(), DELTA);
            assertEquals(Math.PI / 4, setpoint.angle().get().getRadians(), 1e-12);
            assertEquals(3.733, setpoint.norm(), 0.2);
            assertEquals(2.5245058924061974, setpoint.x(), 1e-12);
            assertEquals(2.5206083004022424, setpoint.y(), 0.2);
            assertEquals(3.733, setpoint.theta(), 0.2);
        }
    }

    /** This is pulled from SimulatedDrivingTest, to isolate the problem. */
    @Test
    void courseInvariantRealistic() {
        GlobalVelocityR3 targetSpeed = new GlobalVelocityR3(2, 0, 3.5);

        // not going very fast. note the previous instantaneous robot-relative speed has
        // no "y" component at all, because at the previous time, we had heading of zero
        // (and no speed either).
        GlobalVelocityR3 prevSpeed = new GlobalVelocityR3(0.16333333, 0, 0.28583333);

        // the previous course is exactly zero: this is the first time step after
        // starting.
        assertEquals(0, prevSpeed.angle().get().getRadians(), 1e-12);
        assertEquals(0.16333333, prevSpeed.norm(), 1e-12);
        assertEquals(0.28583333, prevSpeed.theta(), 1e-12);

        // field-relative is +x, field-relative course is zero

        assertEquals(0, targetSpeed.angle().get().getRadians(), 1e-6);
        // the norm is the same as the input
        assertEquals(2, targetSpeed.norm(), 1e-12);
        assertEquals(2, targetSpeed.x(), 1e-12);
        assertEquals(0, targetSpeed.y(), 1e-12);
        assertEquals(3.5, targetSpeed.theta(), 1e-12);

        SwerveLimiter limiter = new SwerveLimiter(logger, KINEMATIC_LIMITS, () -> 12);
        limiter.updateSetpoint(prevSpeed);
        GlobalVelocityR3 setpoint = limiter.apply(targetSpeed);

        assertEquals(0, setpoint.angle().get().getRadians(), 1e-12);
        assertEquals(0.3266666633333334, setpoint.norm(), 1e-12);
        assertEquals(0.5716666631110103, setpoint.theta(), 1e-12);

    }

    @Test
    void motionlessNoOp() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        SwerveLimiter limiter = new SwerveLimiter(logger, unlimited, () -> 12);

        GlobalVelocityR3 target = new GlobalVelocityR3(0, 0, 0);

        assertEquals(0, target.x(), DELTA);
        assertEquals(0, target.y(), DELTA);
        assertEquals(0, target.theta(), DELTA);

        GlobalVelocityR3 prevSetpoint = new GlobalVelocityR3(0, 0, 0);
        limiter.updateSetpoint(prevSetpoint);
        GlobalVelocityR3 setpoint = limiter.apply(target);
        assertEquals(0, setpoint.x(), DELTA);
        assertEquals(0, setpoint.y(), DELTA);
        assertEquals(0, setpoint.theta(), DELTA);

    }

    @Test
    void driveNoOp() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        SwerveLimiter limiter = new SwerveLimiter(logger, unlimited, () -> 12);

        GlobalVelocityR3 target = new GlobalVelocityR3(1, 0, 0);

        assertEquals(1, target.x(), DELTA);
        assertEquals(0, target.y(), DELTA);
        assertEquals(0, target.theta(), DELTA);

        GlobalVelocityR3 prevSetpoint = new GlobalVelocityR3(0, 0, 0);
        limiter.updateSetpoint(prevSetpoint);
        GlobalVelocityR3 setpoint = limiter.apply(target);
        assertEquals(1, setpoint.x(), DELTA);
        assertEquals(0, setpoint.y(), DELTA);
        assertEquals(0, setpoint.theta(), DELTA);

    }

    @Test
    void spinNoOp() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        SwerveLimiter limiter = new SwerveLimiter(logger, unlimited, () -> 12);

        GlobalVelocityR3 target = new GlobalVelocityR3(0, 0, 1);

        assertEquals(0, target.x(), DELTA);
        assertEquals(0, target.y(), DELTA);
        assertEquals(1, target.theta(), DELTA);

        GlobalVelocityR3 prevSetpoint = new GlobalVelocityR3(0, 0, 0);
        limiter.updateSetpoint(prevSetpoint);
        GlobalVelocityR3 setpoint = limiter.apply(target);
        assertEquals(0, setpoint.x(), DELTA);
        assertEquals(0, setpoint.y(), DELTA);
        assertEquals(1, setpoint.theta(), DELTA);
    }

    @Test
    void driveAndSpin() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        SwerveLimiter limiter = new SwerveLimiter(logger, unlimited, () -> 12);

        // spin fast to make the discretization effect larger
        GlobalVelocityR3 target = new GlobalVelocityR3(5, 0, 25);

        assertEquals(5, target.x(), DELTA);
        assertEquals(0, target.y(), DELTA);
        assertEquals(25, target.theta(), DELTA);

        // this should do nothing since the limits are so high
        GlobalVelocityR3 prevSetpoint = new GlobalVelocityR3(0, 0, 0);
        limiter.updateSetpoint(prevSetpoint);
        GlobalVelocityR3 setpoint = limiter.apply(target);
        assertEquals(5, setpoint.x(), DELTA);
        assertEquals(0, setpoint.y(), DELTA);
        assertEquals(25, setpoint.theta(), DELTA);

    }

    // simple accel case: are we limiting the right amount?
    @Test
    void testAccel() {
        // limit accel is 10 m/s^2
        // capsize limit is 24.5 m/s^2
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        assertEquals(24.5, limits.getMaxCapsizeAccelM_S2(), DELTA);
        SwerveLimiter limiter = new SwerveLimiter(logger, limits, () -> 12);

        // initially at rest, wheels facing forward.
        GlobalVelocityR3 setpoint = new GlobalVelocityR3(0, 0, 0);

        // initial setpoint steering is at angle zero

        // desired speed +x
        GlobalVelocityR3 desiredSpeeds = new GlobalVelocityR3(10, 0, 0);

        // the first setpoint should be accel limited: 10 m/s^2, 0.02 sec,
        // so v = 0.2 m/s
        limiter.updateSetpoint(setpoint);
        setpoint = limiter.apply(desiredSpeeds);
        assertEquals(0.2, setpoint.x(), DELTA);
        assertEquals(0, setpoint.y(), DELTA);
        assertEquals(0, setpoint.theta(), DELTA);

        // note this says the angles are all empty which is wrong, they should be the
        // previous values.

        // after 1 second, it's going faster.
        for (int i = 0; i < 50; ++i) {
            setpoint = limiter.apply(desiredSpeeds);
        }
        assertEquals(4.9, setpoint.x(), DELTA);
        assertEquals(0, setpoint.y(), DELTA);
        assertEquals(0, setpoint.theta(), DELTA);
    }

    @Test
    void testNotLimiting() {
        // high centripetal limit to stay out of the way
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        SwerveLimiter limiter = new SwerveLimiter(logger, limits, () -> 12);

        // initially at rest.
        GlobalVelocityR3 setpoint = new GlobalVelocityR3(0, 0, 0);

        // desired speed is feasible, max accel = 10 * dt = 0.02 => v = 0.2
        GlobalVelocityR3 desiredSpeeds = new GlobalVelocityR3(0.2, 0, 0);

        limiter.updateSetpoint(setpoint);
        setpoint = limiter.apply(desiredSpeeds);
        assertEquals(0.2, setpoint.x(), DELTA);
        assertEquals(0, setpoint.y(), DELTA);
        assertEquals(0, setpoint.theta(), DELTA);
    }

    @Test
    void testLimitingALittle() {
        // high centripetal limit to stay out of the way
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        SwerveLimiter limiter = new SwerveLimiter(logger, limits, () -> 12);

        // initially at rest.
        GlobalVelocityR3 setpoint = new GlobalVelocityR3(0, 0, 0);

        // desired speed is double the feasible accel so we should reach it in two
        // iterations.
        GlobalVelocityR3 desiredSpeeds = new GlobalVelocityR3(0.4, 0, 0);

        limiter.updateSetpoint(setpoint);
        setpoint = limiter.apply(desiredSpeeds);
        assertEquals(0.2, setpoint.x(), DELTA);
        assertEquals(0, setpoint.y(), DELTA);
        assertEquals(0, setpoint.theta(), DELTA);

        setpoint = limiter.apply(desiredSpeeds);
        assertEquals(0.4, setpoint.x(), DELTA);
        assertEquals(0, setpoint.y(), DELTA);
        assertEquals(0, setpoint.theta(), DELTA);
    }

    @Test
    void testCase4() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.decelCase();
        SwerveLimiter limiter = new SwerveLimiter(logger, limits, () -> 12);

        // initially moving 0.5 +y
        GlobalVelocityR3 setpoint = new GlobalVelocityR3(0, 0.5, 0);

        // desired state is 1 +x
        final GlobalVelocityR3 desiredSpeeds = new GlobalVelocityR3(1, 0, 0);
        limiter.updateSetpoint(setpoint);
        setpoint = limiter.apply(desiredSpeeds);

        assertEquals(0.146, setpoint.x(), DELTA);
        assertEquals(0.427, setpoint.y(), DELTA);
        assertEquals(0, setpoint.theta(), DELTA);
    }

    /**
     * Fixed target accel run.
     * 
     * The capsize limiter is applied first, and since the goal is *very* infeasible
     * the limit is very low initially. Eventually as speed increases, the capsize
     * limiter scale goes to 1.
     * 
     * You can see that the acceleration limiter starts with current limit, then the
     * back-EMF limit becomes active over 50% speed (the switching point is due to
     * the configuration, and is probably too low). Because the capsize limiter
     * limit is slightly higher than the effect of the current limiter, but both are
     * effectively constant at low speed, the acceleration scale is also initially a
     * constant.
     */
    @Test
    void testSweep() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.likeComp25();
        SwerveLimiter limiter = new SwerveLimiter(logger, limits, () -> 12);
        // target is infeasible and constant
        final GlobalVelocityR3 target = new GlobalVelocityR3(5, 0, 0);
        // start is motionless
        GlobalVelocityR3 setpoint = new GlobalVelocityR3(0, 0, 0);
        limiter.updateSetpoint(setpoint);
        for (int i = 0; i < 100; ++i) {
            if (DEBUG)
                System.out.printf("i %d setpoint %s target %s\n", i, setpoint, target);
            setpoint = limiter.apply(target);
        }
    }

    /**
     * Profiled target accel run.
     * 
     * The main difference between this test and the test above is that profile is
     * unaware of back-EMF limits.
     * 
     * Below 50% speed, the profile produces feasible setpoints, so neither capsize
     * nor acceleration limiter do anything.
     * 
     * Above 50% speed, the profile setpoints are not feasible, which has two
     * effects.
     * 
     * First, the back-EMF limiter starts to affect the target velocity, making it
     * fall behind the profile. The position also falls behind, which affects the
     * lower level controllers.
     * 
     * Second, because the target is behind, the desired acceleration starts to
     * increase (to try to catch up), and the capsize limiter begins to apply.
     * 
     * As the profile reaches "cruise," the capsize limiter stops being
     * active, but the acceleration limiter continues to apply the back-EMF limit.
     * 
     * All this is bad: it would be better for the profile to be aware of the motor
     * physics, and the capsize limit, so that profiles never produced infeasible
     * setpoints.
     */
    @Test
    void testProfile() {
        // profile v and a constraints match the limits
        IncrementalProfile profile = new TrapezoidIncrementalProfile(3, 5, 0.01);
        final Model100 goal = new Model100(5, 0);
        final Model100 initial = new Model100(0, 0);

        final SwerveKinodynamics limits = SwerveKinodynamicsFactory.likeComp25();
        final SwerveLimiter limiter = new SwerveLimiter(logger, limits, () -> 12);

        Control100 profileTarget = initial.control();
        GlobalVelocityR3 target = new GlobalVelocityR3(profileTarget.v(), 0, 0);
        // start is motionless
        GlobalVelocityR3 setpoint = new GlobalVelocityR3(0, 0, 0);
        limiter.updateSetpoint(setpoint);
        for (int i = 0; i < 81; ++i) {
            double accelLimit = SwerveUtil.getAccelLimit(limits, 1, 1, setpoint, target);

            profileTarget = profile.calculate(TimedRobot100.LOOP_PERIOD_S, profileTarget, goal);
            target = new GlobalVelocityR3(profileTarget.v(), 0, 0);
            setpoint = limiter.apply(target);
            if (DEBUG)
                System.out.printf("i %d accelLimit %5.2f setpoint %5.2f target %5.2f\n",
                        i, accelLimit, setpoint.x(), target.x());
        }
    }
}
