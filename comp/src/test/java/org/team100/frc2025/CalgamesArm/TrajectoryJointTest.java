package org.team100.frc2025.CalgamesArm;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GlobalAccelerationR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.prr.AnalyticalJacobian;
import org.team100.lib.motion.prr.Config;
import org.team100.lib.motion.prr.ElevatorArmWristKinematics;
import org.team100.lib.motion.prr.JointAccelerations;
import org.team100.lib.motion.prr.JointVelocities;
import org.team100.lib.state.ControlR3;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.YawRateConstraint;

import edu.wpi.first.math.geometry.Pose2d;

/** How do the joints respond to trajectories? */
public class TrajectoryJointTest {
    private static final boolean DEBUG = false;
    LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    /**
     * How does the smooth cartesian trajectory work in configuration space?
     * 
     * Answer: it's fine, as long as the "reach up"/"reach down" discontinuity is
     * removed.
     * 
     * Acceleration is choppy in both cartesian and configuration, so maybe turn
     * that down a bit.
     * 
     * Charts here:
     * 
     * https://docs.google.com/spreadsheets/d/1yo5gU4NwVDUP8XaGb-7jNOtN6A_7cpX3DWRTfwgKym0/edit?gid=0#gid=0
     */
    @Test
    void homeToL4() {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(log, 1, 1),
                new YawRateConstraint(log, 1, 1));
        TrajectoryPlanner m_planner = new TrajectoryPlanner(c);

        Trajectory100 t = m_planner.restToRest(List.of(
                HolonomicPose2d.make(1, 0, 0, 0),
                HolonomicPose2d.make(1.9, 0.5, 2.5, 2)));

        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(
                0.5, 0.3);
        AnalyticalJacobian J = new AnalyticalJacobian(k);
        if (DEBUG)
            System.out
                    .println(
                            "t, x, y, r, vx, vy, vr, ax, ay, ar, q1, q2, q3, q1dot, q2dot, q3dot, q1ddot, q2ddot, q3ddot");
        for (double tt = 0; tt < t.duration(); tt += 0.02) {
            ControlR3 m = ControlR3.fromTimedPose(t.sample(tt));
            Pose2d p = m.pose();
            GlobalVelocityR3 v = m.velocity();
            GlobalAccelerationR3 a = m.acceleration();
            Config q = k.inverse(p);
            JointVelocities jv = J.inverse(m.model());
            JointAccelerations ja = J.inverseA(m);
            if (DEBUG) {
                System.out.printf(
                        "%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                        tt, p.getX(), p.getY(), p.getRotation().getRadians(), v.x(), v.y(), v.theta(), a.x(),
                        a.y(), a.theta(), q.shoulderHeight(), q.shoulderAngle(), q.wristAngle(), jv.elevator(),
                        jv.shoulder(), jv.wrist(),
                        ja.elevator(), ja.shoulder(), ja.wrist());
            }
        }
    }

}
