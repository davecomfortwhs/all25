package org.team100.lib.trajectory.timing;

import java.util.Optional;

import org.team100.lib.geometry.GlobalAccelerationR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Pose2dWithMotion.MotionDirection;
import org.team100.lib.motion.prr.AnalyticalJacobian;
import org.team100.lib.motion.prr.Config;
import org.team100.lib.motion.prr.ElevatorArmWristKinematics;
import org.team100.lib.motion.prr.JointAccelerations;
import org.team100.lib.motion.prr.JointVelocities;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * For cartesian trajectories executed by non-cartesian mechanisms: each joint
 * has its own velocity and acceleration constraints, which are converted to the
 * cartesian TimingConstraint via the Jacobian.
 * 
 * Acceleration constraints here are constant.
 * 
 * TODO: make acceleration constraint depend on position, to account for
 * gravity, because it's really a motor torque constraint, not an acceleration
 * constraint per se.
 * TODO: make this constraint actually work, I think it's broken.
 */
public class JointConstraint implements TimingConstraint {
    private final ElevatorArmWristKinematics m_k;
    private final AnalyticalJacobian m_j;
    private final JointVelocities m_maxJv;
    private final JointAccelerations m_maxJa;

    public JointConstraint(
            ElevatorArmWristKinematics k,
            AnalyticalJacobian j,
            JointVelocities maxJv,
            JointAccelerations maxJa) {
        m_k = k;
        m_j = j;
        m_maxJv = maxJv;
        m_maxJa = maxJa;
    }

    @Override
    public NonNegativeDouble getMaxVelocity(Pose2dWithMotion state) {
        Pose2d pose = state.getPose();
        MotionDirection motion = state.getMotionDirection();
        // unit vector in cartesian space
        GlobalVelocityR3 v = new GlobalVelocityR3(
                motion.dx(), motion.dy(), motion.dtheta());
        ModelR3 m = new ModelR3(pose, v);

        // corresponding vector in joint space
        JointVelocities qdot = m_j.inverse(m);

        // as a fraction of maxima
        double elevatorScale = Math.abs(qdot.elevator() / m_maxJv.elevator());
        double shoulderScale = Math.abs(qdot.shoulder() / m_maxJv.shoulder());
        double wristScale = Math.abs(qdot.wrist() / m_maxJv.wrist());

        double maxScale = Math.max(elevatorScale, Math.max(shoulderScale, wristScale));

        // scale qdot to the nearest maximum
        JointVelocities maxQdotInMotionDirection = qdot.times(1 / maxScale);

        Config q = m_k.inverse(pose);

        GlobalVelocityR3 maxV = m_j.forward(q, maxQdotInMotionDirection);
        double norm = maxV.norm();
        if (Double.isNaN(norm))
            return new NonNegativeDouble(0);
        return new NonNegativeDouble(norm);
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(
            Pose2dWithMotion state, double velocityM_S) {
        Pose2d pose = state.getPose();
        Optional<Rotation2d> course2 = state.getCourse();

        double c = course2.map(Rotation2d::getCos).orElse(0.0);
        double s = course2.map(Rotation2d::getSin).orElse(0.0);
        double r = state.getHeadingRate();
        double vx = velocityM_S * s;
        double vy = velocityM_S * c;
        double omega = velocityM_S * r;

        // actual cartesian velocity
        GlobalVelocityR3 v = new GlobalVelocityR3(vx, vy, omega);

        Config q = m_k.inverse(pose);
        // actual qdot
        JointVelocities qdot = m_j.inverse(new ModelR3(pose, v));

        // find accel in motion
        GlobalAccelerationR3 unitA = new GlobalAccelerationR3(c, s, r);
        ControlR3 sc = new ControlR3(pose, v, unitA);
        // corresponding a vector in joint space
        JointAccelerations qddot = m_j.inverseA(sc);

        // as a fraction of maxima
        double elevatorScale = Math.abs(qddot.elevator() / m_maxJa.elevator());
        double shoulderScale = Math.abs(qddot.shoulder() / m_maxJa.shoulder());
        double wristScale = Math.abs(qddot.wrist() / m_maxJa.wrist());

        double maxScale = Math.max(elevatorScale, Math.max(shoulderScale, wristScale));

        // scale qddot to the nearest maximum
        JointAccelerations maxQddotInMotionDirection = qddot.times(1 / maxScale);

        GlobalAccelerationR3 fa = m_j.forwardA(q, qdot, maxQddotInMotionDirection);

        double norm = fa.norm();
        if (Double.isNaN(norm))
            return new MinMaxAcceleration(0, 0);
        return new MinMaxAcceleration(-1.0 * norm, 1.0 * norm);
    }

}
