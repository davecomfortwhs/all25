package org.team100.lib.motion.prr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GlobalAccelerationR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Some tests from JacobianTest to verify correctness. */
public class AnalyticalJacobianTest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testForward() {
        final ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        AnalyticalJacobian j = new AnalyticalJacobian(k);
        Config q = new Config(0, 0, 0);
        // extended and motionless
        JointVelocities jv = new JointVelocities(0, 0, 0);
        GlobalVelocityR3 v = j.forward(q, jv);
        assertEquals(0, v.x(), DELTA);
        assertEquals(0, v.y(), DELTA);
        assertEquals(0, v.theta(), DELTA);

        // +shoulder => +y and +theta
        jv = new JointVelocities(0, 1, 0);
        v = j.forward(q, jv);
        assertEquals(0, v.x(), DELTA);
        assertEquals(3, v.y(), DELTA);
        assertEquals(1, v.theta(), DELTA);

        // +wrist => +y and +theta
        jv = new JointVelocities(0, 0, 1);
        v = j.forward(q, jv);
        assertEquals(0, v.x(), DELTA);
        assertEquals(1, v.y(), DELTA);
        assertEquals(1, v.theta(), DELTA);

        // bent at shoulder, +shoulder => -x, +theta
        q = new Config(0, Math.PI / 2, 0);
        jv = new JointVelocities(0, 1, 0);
        v = j.forward(q, jv);
        assertEquals(-3, v.x(), DELTA);
        assertEquals(0, v.y(), DELTA);
        assertEquals(1, v.theta(), DELTA);

        // bent at shoulder and wrist, +wrist => -y, +theta
        q = new Config(0, Math.PI / 2, Math.PI / 2);
        jv = new JointVelocities(0, 0, 1);
        v = j.forward(q, jv);
        assertEquals(0, v.x(), DELTA);
        assertEquals(-1, v.y(), DELTA);
        assertEquals(1, v.theta(), DELTA);
    }

    @Test
    void testInverse() {
        final ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        AnalyticalJacobian j = new AnalyticalJacobian(k);

        Config c = new Config(1, 0, 0);
        Pose2d p = k.forward(c);

        // some example velocities
        // zero velocity
        ModelR3 v = new ModelR3(p);

        JointVelocities jv = j.inverse(v);
        assertEquals(0, jv.elevator(), DELTA);
        assertEquals(0, jv.shoulder(), DELTA);
        assertEquals(0, jv.wrist(), DELTA);

        // +x
        v = new ModelR3(p, new GlobalVelocityR3(1, 0, 0));
        jv = j.inverse(v);
        assertEquals(1, jv.elevator(), DELTA);
        assertEquals(0, jv.shoulder(), DELTA);
        assertEquals(0, jv.wrist(), DELTA);

        // +y
        v = new ModelR3(p, new GlobalVelocityR3(0, 1, 0));
        jv = j.inverse(v);
        assertEquals(0, jv.elevator(), DELTA);
        assertEquals(0.5, jv.shoulder(), DELTA);
        assertEquals(-0.5, jv.wrist(), DELTA);

        // +theta
        v = new ModelR3(p, new GlobalVelocityR3(0, 0, 1));
        jv = j.inverse(v);
        assertEquals(0, jv.elevator(), DELTA);
        assertEquals(-0.5, jv.shoulder(), DELTA);
        assertEquals(1.5, jv.wrist(), DELTA);
    }

    @Test
    void testForwardA() {
        final ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        AnalyticalJacobian j = new AnalyticalJacobian(k);
        Config q = new Config(0, 0, 0);
        // extended, motionless
        JointVelocities qdot = new JointVelocities(0, 0, 0);
        JointAccelerations qddot = new JointAccelerations(0, 0, 0);
        GlobalAccelerationR3 a = j.forwardA(q, qdot, qddot);
        assertEquals(0, a.x(), DELTA);
        assertEquals(0, a.y(), DELTA);
        assertEquals(0, a.theta(), DELTA);

        // +elevator => +x
        q = new Config(0, 0, 0);
        qdot = new JointVelocities(0, 0, 0);
        qddot = new JointAccelerations(1, 0, 0);
        a = j.forwardA(q, qdot, qddot);
        assertEquals(1, a.x(), DELTA);
        assertEquals(0, a.y(), DELTA);
        assertEquals(0, a.theta(), DELTA);

        // +shoulder => +y, +theta
        q = new Config(0, 0, 0);
        qdot = new JointVelocities(0, 0, 0);
        qddot = new JointAccelerations(0, 1, 0);
        a = j.forwardA(q, qdot, qddot);
        assertEquals(0, a.x(), DELTA);
        assertEquals(3, a.y(), DELTA);
        assertEquals(1, a.theta(), DELTA);

        // +wrist => +y, +theta
        q = new Config(0, 0, 0);
        qdot = new JointVelocities(0, 0, 0);
        qddot = new JointAccelerations(0, 0, 1);
        a = j.forwardA(q, qdot, qddot);
        assertEquals(0, a.x(), DELTA);
        assertEquals(1, a.y(), DELTA);
        assertEquals(1, a.theta(), DELTA);

        // shoulder bent, +shoulder => -x, +theta
        q = new Config(0, Math.PI / 2, 0);
        qdot = new JointVelocities(0, 0, 0);
        qddot = new JointAccelerations(0, 1, 0);
        a = j.forwardA(q, qdot, qddot);
        assertEquals(-3, a.x(), DELTA);
        assertEquals(0, a.y(), DELTA);
        assertEquals(1, a.theta(), DELTA);

        // shoulder and wrist bent, +wrist => -y, +theta
        q = new Config(0, Math.PI / 2, Math.PI / 2);
        qdot = new JointVelocities(0, 0, 0);
        qddot = new JointAccelerations(0, 0, 1);
        a = j.forwardA(q, qdot, qddot);
        assertEquals(0, a.x(), DELTA);
        assertEquals(-1, a.y(), DELTA);
        assertEquals(1, a.theta(), DELTA);
    }

    @Test
    void testInverseA() {
        final ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        AnalyticalJacobian j = new AnalyticalJacobian(k);

        // extended, motionless
        Config c = new Config(0, 0, 0);
        Pose2d p = k.forward(c);
        GlobalVelocityR3 v = new GlobalVelocityR3(0, 0, 0);
        ControlR3 m = new ControlR3(p, v, new GlobalAccelerationR3(0, 0, 0));
        JointAccelerations ja = j.inverseA(m);
        assertEquals(0, ja.elevator(), DELTA);
        assertEquals(0, ja.shoulder(), DELTA);
        assertEquals(0, ja.wrist(), DELTA);

        // +x => +elevator
        c = new Config(0, 0, 0);
        p = k.forward(c);
        v = new GlobalVelocityR3(0, 0, 0);
        m = new ControlR3(p, v, new GlobalAccelerationR3(1, 0, 0));
        ja = j.inverseA(m);
        assertEquals(1, ja.elevator(), DELTA);
        assertEquals(0, ja.shoulder(), DELTA);
        assertEquals(0, ja.wrist(), DELTA);

        // +y => +shoulder and -wrist (because zero theta)
        c = new Config(0, 0, 0);
        p = k.forward(c);
        v = new GlobalVelocityR3(0, 0, 0);
        m = new ControlR3(p, v, new GlobalAccelerationR3(0, 1, 0));
        ja = j.inverseA(m);
        assertEquals(0, ja.elevator(), DELTA);
        assertEquals(0.5, ja.shoulder(), DELTA);
        assertEquals(-0.5, ja.wrist(), DELTA);

        // +theta => -shoulder, +wrist (because no translation)
        c = new Config(0, 0, 0);
        p = k.forward(c);
        v = new GlobalVelocityR3(0, 0, 0);
        m = new ControlR3(p, v, new GlobalAccelerationR3(0, 0, 1));
        ja = j.inverseA(m);
        assertEquals(0, ja.elevator(), DELTA);
        assertEquals(-0.5, ja.shoulder(), DELTA);
        assertEquals(1.5, ja.wrist(), DELTA);

        // bent shoulder, +x => +elevator only
        // using 45 deg because of singularity at 90
        c = new Config(0, Math.PI / 4, 0);
        p = k.forward(c);
        v = new GlobalVelocityR3(0, 0, 0);
        m = new ControlR3(p, v, new GlobalAccelerationR3(1, 0, 0));
        ja = j.inverseA(m);
        assertEquals(1, ja.elevator(), DELTA);
        assertEquals(0, ja.shoulder(), DELTA);
        assertEquals(0, ja.wrist(), DELTA);

        // bent shoulder, +y => +elevator, +shoulder, -wrist (because no theta)
        c = new Config(0, Math.PI / 4, 0);
        // using 45 deg because of singularity at 90
        p = k.forward(c);
        v = new GlobalVelocityR3(0, 0, 0);
        m = new ControlR3(p, v, new GlobalAccelerationR3(0, 1, 0));
        ja = j.inverseA(m);
        assertEquals(1, ja.elevator(), DELTA);
        assertEquals(0.707, ja.shoulder(), DELTA);
        assertEquals(-0.707, ja.wrist(), DELTA);

        // bent shoulder and wrist, +y => +elevator, +shoulder, -wrist
        c = new Config(0, Math.PI / 4, Math.PI / 4);
        // using 45 deg because of singularity at 90
        p = k.forward(c);
        v = new GlobalVelocityR3(0, 0, 0);
        m = new ControlR3(p, v, new GlobalAccelerationR3(0, 1, 0));
        ja = j.inverseA(m);
        assertEquals(1, ja.elevator(), DELTA);
        assertEquals(0.707, ja.shoulder(), DELTA);
        assertEquals(-0.707, ja.wrist(), DELTA);
    }

    @Test
    void testTrajectory() {
        final ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        AnalyticalJacobian j = new AnalyticalJacobian(k);

        TrajectoryPlanner planner = new TrajectoryPlanner(List.of(new ConstantConstraint(logger, 1, 1)));
        Pose2d start = new Pose2d(1, -1, Rotation2d.kZero);
        Pose2d end = new Pose2d(2, 1, Rotation2d.k180deg);
        Trajectory100 t = planner.restToRest(start, end);
        double d = t.duration();
        double dt = d / 20;
        for (double time = 0; time < d; time += dt) {
            TimedPose tp = t.sample(time);
            ModelR3 sm = ModelR3.fromTimedPose(tp);
            Pose2d p = sm.pose();
            GlobalVelocityR3 v = sm.velocity();
            Config c = k.inverse(p);
            JointVelocities jv = j.inverse(sm);
            if (DEBUG)
                System.out.printf(
                        "s (%5.2f) pose(%5.2f %5.2f %5.2f) conf(%5.2f %5.2f %5.2f) tv(%5.2f %5.2f %5.2f) jv(%5.2f %5.2f %5.2f)\n",
                        time,
                        p.getX(), p.getY(), p.getRotation().getRadians(),
                        c.shoulderHeight(), c.shoulderAngle(), c.wristAngle(),
                        v.x(), v.y(), v.theta(),
                        jv.elevator(), jv.shoulder(), jv.wrist());
        }
    }
}
