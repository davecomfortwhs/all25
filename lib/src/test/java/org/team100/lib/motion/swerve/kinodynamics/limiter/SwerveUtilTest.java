package org.team100.lib.motion.swerve.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.motion.swerve.Fixture;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamicsFactory;

class SwerveUtilTest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;

    @Test
    void testIsAccel() {
        // hard left turn is not accel
        assertFalse(SwerveUtil.isAccel(
                new GlobalVelocityR3(1, 0, 0),
                new GlobalVelocityR3(0, 1, 0)));
        // speed up veering left
        assertTrue(SwerveUtil.isAccel(
                new GlobalVelocityR3(0.5, 0.5, 0),
                new GlobalVelocityR3(0, 1, 0)));
    }

    @Test
    void testGetMaxVelStepWithVelocityDependentAccel() {
        // available acceleration is not always the max.
        // a motor without current limiting has a straight declining torque curve
        // a motor with current limiting has a constant torque curve for awhile
        // hm, how to get the motor model in here?
    }

    @Test
    void testAccelLimit1() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forRealisticTest();
        assertEquals(10, limits.getMaxDriveAccelerationM_S2(), DELTA);
        double accelLimit = SwerveUtil.getAccelLimit(limits, 1, 1,
                new GlobalVelocityR3(0, 0, 0),
                new GlobalVelocityR3(1, 0, 0));
        // low speed, current limited.
        assertEquals(10, accelLimit, DELTA);
    }

    @Test
    void testAccelLimit2() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forRealisticTest();
        assertEquals(10, limits.getMaxDriveAccelerationM_S2(), DELTA);
        assertEquals(5, limits.getMaxDriveVelocityM_S(), DELTA);
        double accelLimit = SwerveUtil.getAccelLimit(limits, 1, 1,
                new GlobalVelocityR3(4.9, 0, 0),
                new GlobalVelocityR3(5, 0, 0));
        // near top speed, EMF-limited
        assertEquals(0.2, accelLimit, DELTA);
    }

    @Test
    void testGetAccelLimit() throws IOException {
        // this is to figure out why the Oscillate test isn't returning
        // exactly the right result
        SwerveKinodynamics limits = new Fixture().swerveKinodynamics;
        assertEquals(1, limits.getMaxDriveAccelerationM_S2(), DELTA);
        double accelLimit = SwerveUtil.getAccelLimit(limits, 1, 1,
                new GlobalVelocityR3(0.92, 0, 0),
                new GlobalVelocityR3(0.94, 0, 0));
        assertEquals(0.8, accelLimit, DELTA);
    }

    @Test
    void testMinAccel() throws IOException {
        // this is to figure out why the Oscillate test isn't returning
        // exactly the right result
        SwerveKinodynamics limits = new Fixture().swerveKinodynamics;
        // the test asks for 1 m/s/s
        assertEquals(1, limits.getMaxDriveAccelerationM_S2(), DELTA);
        // the problem is that the maximum possible velocity is right at the
        // maximum commanded velocity, so the motor can't execute the constant
        // accel command.
        assertEquals(1, limits.getMaxDriveVelocityM_S(), DELTA);
        assertEquals(10, limits.getStallAccelerationM_S2(), DELTA);
        // this returns 0.8 which is wrong
        double accelLimit = SwerveUtil.minAccel(limits, 1, 1, 0.92);
        assertEquals(0.8, accelLimit, DELTA);
    }

    @Test
    void simMinAccel() {
        // simulate full-throttle to see the exponential curve.
        // https://docs.google.com/spreadsheets/d/1k-g8_blQP3X1RNtjFQgk1CJXyzzvNLNuUbWhaNLduvw
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forRealisticTest();
        double v = 0;
        final double dt = 0.02;
        for (double t = 0; t < 3; t += dt) {
            if (DEBUG)
                System.out.printf("%5.3f %5.3f\n", t, v);
            double a = SwerveUtil.minAccel(limits, 1, 1, v);
            v += dt * a;
        }
    }

    @Test
    void testJerkLimit() {

    }
}
