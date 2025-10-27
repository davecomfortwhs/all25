package org.team100.lib.controller.r3;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GlobalDeltaR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Pose2dWithMotion.MotionDirection;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.Control100;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.Model100;
import org.team100.lib.state.ModelR3;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class FullStateControllerR3Test implements Timeless {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testAtRest() {
        FullStateControllerR3 c = ControllerFactoryR3.test2(logger);
        assertFalse(c.atReference());
        GlobalVelocityR3 t = c.calculate(
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
        assertEquals(0, t.x(), DELTA);
        assertEquals(0, t.y(), DELTA);
        assertEquals(0, t.theta(), DELTA);
        assertTrue(c.atReference());
    }

    @Test
    void testFar() {
        FullStateControllerR3 c = ControllerFactoryR3.test2(logger);
        assertFalse(c.atReference());
        // no velocity, no feedforward
        GlobalVelocityR3 t = c.calculate(
                new ModelR3(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ModelR3(
                        new Model100(1, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ControlR3(
                        new Control100(1, 0),
                        new Control100(0, 0),
                        new Control100(0, 0)));
        // 1m error so dx should be K*e = 1
        assertEquals(4, t.x(), DELTA);
        assertEquals(0, t.y(), DELTA);
        assertEquals(0, t.theta(), DELTA);
        assertFalse(c.atReference());
    }

    @Test
    void testFast() {
        FullStateControllerR3 c = ControllerFactoryR3.test2(logger);
        assertFalse(c.atReference());
        GlobalVelocityR3 t = c.calculate(
                new ModelR3(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ModelR3(
                        new Model100(0, 1), // produces error = 1
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ControlR3(
                        new Control100(0, 1), // produces FF = 1
                        new Control100(0, 0),
                        new Control100(0, 0)));
        // position err is zero but velocity error is 1 and feedforward is also 1 so dx
        // should be FF + K*e = 2
        assertEquals(1.25, t.x(), DELTA);
        assertEquals(0, t.y(), DELTA);
        assertEquals(0, t.theta(), DELTA);
        assertFalse(c.atReference());
    }

    @Test
    void testOnTrack() {
        FullStateControllerR3 c = ControllerFactoryR3.test2(logger);
        assertFalse(c.atReference());
        GlobalVelocityR3 t = c.calculate(
                new ModelR3(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ModelR3(
                        new Model100(-1, 0.5), // position + velocity error
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ControlR3(
                        new Control100(-1, 0.5), // velocity reference
                        new Control100(0, 0),
                        new Control100(0, 0)));
        // position and velocity controls are opposite, so just cruise
        assertEquals(-3.375, t.x(), DELTA);
        assertEquals(0, t.y(), DELTA);
        assertEquals(0, t.theta(), DELTA);
        assertFalse(c.atReference());
    }

    @Test
    void testAllAxes() {
        FullStateControllerR3 c = ControllerFactoryR3.test2(logger);
        assertFalse(c.atReference());
        // none of these have any velocity so there's no feedforward.
        GlobalVelocityR3 t = c.calculate(
                new ModelR3(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new ModelR3(
                        new Model100(1, 0),
                        new Model100(2, 0),
                        new Model100(3, 0)),
                new ControlR3(
                        new Control100(2, 0),
                        new Control100(4, 0),
                        new Control100(6, 0)));
        // 1m error so dx should be K*e = 1
        assertEquals(4, t.x(), DELTA);
        assertEquals(8, t.y(), DELTA);
        assertEquals(12, t.theta(), DELTA);
        assertFalse(c.atReference());
    }

    @Test
    void testRotation() {
        FullStateControllerR3 c = ControllerFactoryR3.test2(logger);
        assertFalse(c.atReference());
        GlobalVelocityR3 t = c.calculate(
                new ModelR3(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(3, 0)),
                new ModelR3(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(-3, 0)),
                new ControlR3(
                        new Control100(0, 0),
                        new Control100(0, 0),
                        new Control100(-3, 0)));
        assertEquals(0, t.x(), DELTA);
        assertEquals(0, t.y(), DELTA);
        // we want to rotate +
        assertEquals(1.133, t.theta(), DELTA);
        assertFalse(c.atReference());
    }

    @Test
    void testErrZero() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02,
                0.01, 0.02);
        ModelR3 measurement = new ModelR3();
        ModelR3 currentReference = ModelR3
                .fromTimedPose(new TimedPose(new Pose2dWithMotion(new Pose2d()), 0, 0, 0));
        GlobalDeltaR3 err = controller.positionError(measurement, currentReference);
        assertEquals(0, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    @Test
    void testErrXAhead() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02,
                0.01, 0.02);
        ModelR3 measurement = new ModelR3(new Pose2d(1, 0, new Rotation2d()));
        ModelR3 currentReference = ModelR3
                .fromTimedPose(new TimedPose(new Pose2dWithMotion(new Pose2d()), 0, 0, 0));
        GlobalDeltaR3 err = controller.positionError(measurement, currentReference);
        assertEquals(-1, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    @Test
    void testErrXBehind() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02,
                0.01, 0.02);
        ModelR3 measurement = new ModelR3(new Pose2d(0, 0, new Rotation2d()));
        ModelR3 currentReference = ModelR3
                .fromTimedPose(new TimedPose(new Pose2dWithMotion(new Pose2d(1, 0, new Rotation2d())), 0, 0, 0));
        GlobalDeltaR3 err = controller.positionError(measurement, currentReference);
        assertEquals(1, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    /** Rotation should not matter. */
    @Test
    void testErrXAheadWithRotation() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02,
                0.01, 0.02);
        ModelR3 measurement = new ModelR3(new Pose2d(1, 0, new Rotation2d(1)));
        ModelR3 currentReference = ModelR3
                .fromTimedPose(new TimedPose(new Pose2dWithMotion(new Pose2d(0, 0, new Rotation2d(1))), 0, 0, 0));
        GlobalDeltaR3 err = controller.positionError(measurement, currentReference);
        assertEquals(-1, err.getX(), 0.001);
        assertEquals(0, err.getY(), 0.001);
        assertEquals(0, err.getRotation().getRadians(), 0.001);
    }

    @Test
    void testErrorAhead() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02,
                0.01, 0.02);
        // measurement is at the origin, facing ahead
        ModelR3 measurement = new ModelR3(new Pose2d());
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);

        // setpoint is also at the origin
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(),
                motionDirection,
                0, // no curvature
                0); // no change in curvature
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        // we're exactly on the setpoint so zero error
        ModelR3 currentReference = ModelR3.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        GlobalDeltaR3 positionError = controller.positionError(measurement, currentReference);
        assertEquals(0, positionError.getX(), DELTA);
        assertEquals(0, positionError.getY(), DELTA);
        assertEquals(0, positionError.getRadians(), DELTA);
    }

    @Test
    void testErrorSideways() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 2.4, 2.4, 0.0, 0.0, 0.01, 0.02,
                0.01, 0.02);
        // measurement is at the origin, facing down y
        ModelR3 measurement = new ModelR3(new Pose2d(0, 0, Rotation2d.kCCW_Pi_2));
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // setpoint is +x, facing down y
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(1, 0, Rotation2d.kCCW_Pi_2),
                motionDirection,
                0, // no curvature
                0); // no change in curvature
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        ModelR3 currentReference = ModelR3.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        GlobalDeltaR3 positionError = controller.positionError(measurement, currentReference);
        assertEquals(1, positionError.getX(), DELTA);
        assertEquals(0, positionError.getY(), DELTA);
        assertEquals(0, positionError.getRadians(), DELTA);
    }

    @Test
    void testVelocityErrorZero() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // measurement position doesn't matter, rotation here matches velocity below
        ModelR3 measurement = new ModelR3(
                new Pose2d(1, 2, new Rotation2d(Math.PI)),
                new GlobalVelocityR3(1, 0, 0));
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // setpoint is also at the origin
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(),
                motionDirection,
                0, // no curvature
                0); // no change in curvature
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        ModelR3 currentReference = ModelR3.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        GlobalVelocityR3 error = controller.velocityError(measurement, currentReference);
        // we're exactly on the setpoint so zero error
        assertEquals(0, error.x(), DELTA);
        assertEquals(0, error.y(), DELTA);
        assertEquals(0, error.theta(), DELTA);
    }

    @Test
    void testVelocityErrorAhead() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // measurement is at the origin, facing ahead
        // measurement is the wrong velocity
        ModelR3 measurement = new ModelR3(
                new Pose2d(),
                new GlobalVelocityR3(0, 1, 0));
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // at the origin
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(),
                motionDirection,
                0, // no curvature
                0); // no change in curvature
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;

        ModelR3 currentReference = ModelR3.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        GlobalVelocityR3 error = controller.velocityError(measurement, currentReference);
        // error should include both components
        assertEquals(1, error.x(), DELTA);
        assertEquals(-1, error.y(), DELTA);
        assertEquals(0, error.theta(), DELTA);
    }

    @Test
    void testFeedForwardAhead() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // setpoint is also at the origin
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(),
                motionDirection,
                0, // no curvature
                0);// no change in curvature
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be straight ahead, no rotation.
        ControlR3 nextReference = ControlR3.fromTimedPose(setpoint);
        GlobalVelocityR3 speeds = controller.feedforward(nextReference);
        assertEquals(1, speeds.x(), DELTA);
        assertEquals(0, speeds.y(), DELTA);
        assertEquals(0, speeds.theta(), DELTA);
    }

    @Test
    void testFeedForwardSideways() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // setpoint is the same
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(0, 0, Rotation2d.kCCW_Pi_2),
                motionDirection,
                0, // no curvature
                0); // no change in curvature
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be -y, robot relative, no rotation.
        ControlR3 nextReference = ControlR3.fromTimedPose(setpoint);
        GlobalVelocityR3 speeds = controller.feedforward(nextReference);
        assertEquals(1, speeds.x(), DELTA);
        assertEquals(0, speeds.y(), DELTA);
        assertEquals(0, speeds.theta(), DELTA);
    }

    @Test
    void testFeedForwardTurning() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // motion is tangential to the x axis but turning left
        MotionDirection motionDirection = new MotionDirection(1, 0, 1);
        // setpoint is also at the origin
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(),
                motionDirection,
                1, // driving and turning
                0); // no change in curvature
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        ControlR3 nextReference = ControlR3.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        GlobalVelocityR3 speeds = controller.feedforward(nextReference);
        // feedforward should be ahead and rotating.
        assertEquals(1, speeds.x(), DELTA);
        assertEquals(0, speeds.y(), DELTA);
        assertEquals(1, speeds.theta(), DELTA);
    }

    @Test
    void testFeedbackAhead() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // measurement is at the origin, facing ahead
        Pose2d currentState = new Pose2d();
        // setpoint is also at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        ModelR3 measurement = new ModelR3(
                currentState,
                new GlobalVelocityR3(1, 0, 0));
        // feedforward should be straight ahead, no rotation.
        ModelR3 currentReference = ModelR3.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        GlobalVelocityR3 speeds = controller.positionFeedback(measurement, currentReference);
        // we're exactly on the setpoint so zero feedback
        assertEquals(0, speeds.x(), DELTA);
        assertEquals(0, speeds.y(), DELTA);
        assertEquals(0, speeds.theta(), DELTA);
    }

    @Test
    void testFeedbackAheadPlusY() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // measurement is plus-Y, facing ahead
        Pose2d currentState = new Pose2d(0, 1, Rotation2d.kZero);
        // setpoint is at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        ModelR3 measurement = new ModelR3(
                currentState,
                new GlobalVelocityR3(1, 0, 0));
        // feedforward should be straight ahead, no rotation.
        ModelR3 currentReference = ModelR3.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        GlobalVelocityR3 speeds = controller.positionFeedback(measurement, currentReference);
        // setpoint should be negative y
        assertEquals(0, speeds.x(), DELTA);
        assertEquals(-1, speeds.y(), DELTA);
        assertEquals(0, speeds.theta(), DELTA);
    }

    @Test
    void testFeedbackAheadPlusTheta() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // measurement is plus-theta
        Pose2d currentState = new Pose2d(0, 0, new Rotation2d(1.0));
        // setpoint is also at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        ModelR3 measurement = new ModelR3(
                currentState,
                new GlobalVelocityR3(1, 0, 0));
        // feedforward should be straight ahead, no rotation.
        ModelR3 currentReference = ModelR3.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        GlobalVelocityR3 speeds = controller.positionFeedback(measurement, currentReference);
        // robot is on the setpoint in translation
        // but needs negative rotation
        // setpoint should be negative theta
        assertEquals(0, speeds.x(), DELTA);
        assertEquals(0, speeds.y(), DELTA);
        assertEquals(-1, speeds.theta(), DELTA);
    }

    @Test
    void testFeedbackSideways() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);

        // measurement is at the origin, facing down the y axis
        Pose2d currentState = new Pose2d(0, 0, Rotation2d.kCCW_Pi_2);
        // setpoint is the same
        Pose2d setpointPose = new Pose2d(0, 0, Rotation2d.kCCW_Pi_2);
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        ModelR3 measurement = new ModelR3(
                currentState,
                new GlobalVelocityR3(1, 0, 0));
        ModelR3 currentReference = ModelR3.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        GlobalVelocityR3 speeds = controller.positionFeedback(measurement, currentReference);
        // on target
        assertEquals(0, speeds.x(), DELTA);
        assertEquals(0, speeds.y(), DELTA);
        assertEquals(0, speeds.theta(), DELTA);
    }

    @Test
    void testFeedbackSidewaysPlusY() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 1, 1, 0, 0, 0.01, 0.02, 0.01,
                0.02);
        // measurement is plus-y, facing down the y axis
        Pose2d currentState = new Pose2d(0, 1, Rotation2d.kCCW_Pi_2);
        // setpoint is parallel at the origin
        Pose2d setpointPose = new Pose2d(0, 0, Rotation2d.kCCW_Pi_2);
        // motion is in a straight line, down the x axis
        MotionDirection motionDirection = new MotionDirection(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        ModelR3 measurement = new ModelR3(
                currentState,
                new GlobalVelocityR3(1, 0, 0));
        ModelR3 currentReference = ModelR3.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        GlobalVelocityR3 speeds = controller.positionFeedback(measurement, currentReference);
        // feedback is -y field relative
        assertEquals(0, speeds.x(), DELTA);
        assertEquals(-1, speeds.y(), DELTA);
        assertEquals(0, speeds.theta(), DELTA);
    }

    @Test
    void testFullFeedbackAhead() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 1, 1, 1, 1, 0.01, 0.02, 0.01,
                0.02);
        // measurement is at the origin, facing ahead
        Pose2d currentState = new Pose2d();
        // setpoint is also at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is in a straight line, down the x axis
        MotionDirection fieldRelativeMotionDirection = new MotionDirection(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                fieldRelativeMotionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        // motion is on setpoint
        ModelR3 measurement = new ModelR3(
                currentState,
                new GlobalVelocityR3(1, 0, 0));
        ModelR3 currentReference = ModelR3.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        GlobalVelocityR3 speeds = controller.fullFeedback(measurement, currentReference);
        // we're exactly on the setpoint so zero feedback
        assertEquals(0, speeds.x(), DELTA);
        assertEquals(0, speeds.y(), DELTA);
        assertEquals(0, speeds.theta(), DELTA);
    }

    @Test
    void testFullFeedbackSideways() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 1, 1, 1, 1, 0.01, 0.02, 0.01,
                0.02);

        // measurement is at the origin, facing +y
        Pose2d currentPose = new Pose2d(0, 0, Rotation2d.kCCW_Pi_2);
        // setpoint postion is the same
        Pose2d setpointPose = new Pose2d(0, 0, Rotation2d.kCCW_Pi_2);
        // motion is in a straight line, down the x axis
        MotionDirection fieldRelativeMotionDirection = new MotionDirection(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                fieldRelativeMotionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        // measurement is too slow
        ModelR3 measurement = new ModelR3(
                currentPose,
                new GlobalVelocityR3(0.5, 0, 0));
        ModelR3 currentReference = ModelR3.fromTimedPose(new TimedPose(state, t, velocity, acceleration));
        GlobalVelocityR3 speeds = controller.fullFeedback(measurement, currentReference);
        // speed up
        assertEquals(0.5, speeds.x(), DELTA);
        assertEquals(0, speeds.y(), DELTA);
        assertEquals(0, speeds.theta(), DELTA);
    }

    @Test
    void testFullFeedbackSidewaysWithRotation() {
        FullStateControllerR3 controller = new FullStateControllerR3(logger, 1, 1, 1, 1, 0.01, 0.02, 0.01,
                0.02);

        // measurement is ahead in x and y and theta
        // measurement is too slow
        ModelR3 measurement = new ModelR3(
                new Pose2d(0.1, 0.1,
                        Rotation2d.kCCW_Pi_2.plus(new Rotation2d(0.1))),
                new GlobalVelocityR3(0.5, 0, 0));

        // setpoint postion is ahead in x and y and theta
        Pose2d setpointPose = new Pose2d(0, 0, Rotation2d.kCCW_Pi_2);
        // motion is in a straight line, down the x axis
        MotionDirection fieldRelativeMotionDirection = new MotionDirection(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                fieldRelativeMotionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;

        ModelR3 currentReference = ModelR3.fromTimedPose(new TimedPose(state, t, velocity, acceleration));

        // feedforward should be straight ahead, no rotation.

        GlobalVelocityR3 positionFeedback = controller.positionFeedback(
                measurement, currentReference);
        // field-relative x is ahead
        assertEquals(-0.1, positionFeedback.x(), DELTA);
        // field-relative y is ahead
        assertEquals(-0.1, positionFeedback.y(), DELTA);
        // pull back theta
        assertEquals(-0.1, positionFeedback.theta(), DELTA);

        GlobalVelocityR3 velocityFeedback = controller.velocityFeedback(
                measurement,
                currentReference);

        assertEquals(0.5, velocityFeedback.x(), DELTA);
        assertEquals(0, velocityFeedback.y(), DELTA);
        assertEquals(0, velocityFeedback.theta(), DELTA);

        GlobalVelocityR3 speeds = controller.fullFeedback(
                measurement,
                currentReference);
        // this is just the sum
        assertEquals(0.4, speeds.x(), DELTA);
        assertEquals(-0.1, speeds.y(), DELTA);
        assertEquals(-0.1, speeds.theta(), DELTA);
    }
}
