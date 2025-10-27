package org.team100.lib.motion.servo;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.controller.r1.Feedback100;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.encoder.MockRotaryPositionSensor;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motor.MockBareMotor;
import org.team100.lib.profile.incremental.IncrementalProfile;
import org.team100.lib.profile.incremental.TrapezoidIncrementalProfile;
import org.team100.lib.profile.incremental.TrapezoidProfileWPI;
import org.team100.lib.reference.r1.IncrementalProfileReferenceR1;
import org.team100.lib.testing.Timeless;

class AngularPositionProfileTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    private final MockBareMotor motor;
    private final RotaryMechanism mech;
    private final MockRotaryPositionSensor sensor;
    private final Feedback100 feedback2;
    IncrementalProfileReferenceR1 ref;
    private OnboardAngularPositionServo servo;

    public AngularPositionProfileTest() {
        motor = new MockBareMotor(Feedforward100.makeSimple(logger));
        sensor = new MockRotaryPositionSensor();
        mech = new RotaryMechanism(
                logger, motor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        feedback2 = new PIDFeedback(logger, 5, 0, 0, false, 0.05, 1);
    }

    /**
     * Profile invariant to support refactoring the servo. This is the WPILib
     * TrapezoidalProfile.
     */
    @Test
    void testTrapezoid() {
        final IncrementalProfile profile = new TrapezoidProfileWPI(1, 1);
        ref = new IncrementalProfileReferenceR1(profile, 0.05, 0.05);
        servo = new OnboardAngularPositionServo(
                logger,
                mech,
                ref,
                feedback2);
        servo.reset();

        verifyTrapezoid();
    }

    @Test
    void testProfile() {
        final IncrementalProfile profile = new TrapezoidIncrementalProfile(1, 1, 0.05);
        ref = new IncrementalProfileReferenceR1(profile, 0.05, 0.05);
        servo = new OnboardAngularPositionServo(
                logger,
                mech,
                ref,
                feedback2);
        servo.reset();
        verifyTrapezoid();
    }

    private void verifyTrapezoid() {
        verify(0.125, 0.005, 0.100);
        verify(0.238, 0.020, 0.200);
        verify(0.344, 0.045, 0.300);
        verify(0.447, 0.080, 0.400);
        verify(0.548, 0.125, 0.500);
        verify(0.649, 0.180, 0.600);
        verify(0.750, 0.245, 0.700);
        verify(0.850, 0.320, 0.800);
        verify(0.950, 0.405, 0.900);
        verify(1.000, 0.500, 1.000);
        verify(0.925, 0.595, 0.900);
        verify(0.787, 0.680, 0.800);
        verify(0.669, 0.755, 0.700);
        verify(0.559, 0.820, 0.600);
        verify(0.455, 0.875, 0.500);
        verify(0.352, 0.920, 0.400);
        verify(0.251, 0.955, 0.300);
        verify(0.151, 0.980, 0.200);
        verify(0.050, 0.995, 0.100);
        verify(-0.050, 1.000, 0.000);
        verify(-0.025, 1.000, 0.000);
    }

    private void verify(double motorVelocity, double setpointPosition, double setpointVelocity) {
        sensor.angle += motor.velocity * TimedRobot100.LOOP_PERIOD_S;
        // spin for 100ms
        for (int i = 0; i < 5; ++i) {
            servo.setPositionProfiled(1, 0);
            stepTime();
        }
        // useful to fix up the examples above
        if (DEBUG)
            System.out.printf("verify(%5.3f, %5.3f, %5.3f);\n", motor.velocity,
                    servo.m_nextUnwrappedSetpoint.x(), servo.m_nextUnwrappedSetpoint.v());
        assertEquals(setpointPosition, servo.m_nextUnwrappedSetpoint.x(), DELTA, "position");
        assertEquals(setpointVelocity, servo.m_nextUnwrappedSetpoint.v(), DELTA, "velocity");
    }
}
