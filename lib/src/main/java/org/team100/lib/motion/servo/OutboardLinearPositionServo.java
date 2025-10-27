package org.team100.lib.motion.servo;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.SetpointsR1;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * Profiled or direct position control using the feedback controller in the
 * motor controller hardware.
 */
public class OutboardLinearPositionServo implements LinearPositionServo {
    private final LinearMechanism m_mechanism;
    private final ProfileReferenceR1 m_ref;
    private final double m_positionTolerance; // 0.01
    private final double m_velocityTolerance; // 0.01

    private final DoubleLogger m_log_goal;
    private final DoubleLogger m_log_ff_torque;
    private final Control100Logger m_log_control;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;

    /** Null if there's no current profile. */
    private Model100 m_goal;
    private Control100 m_nextSetpoint;

    public OutboardLinearPositionServo(
            LoggerFactory parent,
            LinearMechanism mechanism,
            ProfileReferenceR1 ref,
            double positionTolerance,
            double velocityTolerance) {
        LoggerFactory child = parent.type(this);
        m_mechanism = mechanism;
        m_ref = ref;
        m_positionTolerance = positionTolerance;
        m_velocityTolerance = velocityTolerance;
        m_log_goal = child.doubleLogger(Level.COMP, "goal (m)");
        m_log_ff_torque = child.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_control = child.control100Logger(Level.COMP, "control (m)");
        m_log_position = child.doubleLogger(Level.COMP, "position (m)");
        m_log_velocity = child.doubleLogger(Level.COMP, "velocity (m_s)");
    }

    @Override
    public void reset() {
        // using the current velocity sometimes includes a whole lot of noise, and then
        // the profile tries to follow that noise. so instead, use zero.
        Control100 measurement = new Control100(getPosition(), 0);
        m_nextSetpoint = measurement;
        // reference is initalized with measurement only here.
        m_ref.setGoal(measurement.model());
        m_ref.init(measurement.model());
    }

    /** Resets the profile if necessary */
    @Override
    public void setPositionProfiled(double goalM, double feedForwardTorqueNm) {
        m_log_goal.log(() -> goalM);
        Model100 goal = new Model100(goalM, 0);

        if (!goal.near(m_goal, m_positionTolerance, m_velocityTolerance)) {
            m_goal = goal;
            m_ref.setGoal(goal);
            if (m_nextSetpoint == null) {
                // erased by dutycycle control
                m_nextSetpoint = new Control100(getPosition(), 0);
            }
            // initialize with the setpoint, not the measurement, to avoid noise.
            m_ref.init(m_nextSetpoint.model());
        }
        actuate(m_ref.get(), feedForwardTorqueNm);
    }

    /** Invalidates the current profile */
    @Override
    public void setPositionDirect(SetpointsR1 setpoints, double feedForwardTorqueNm) {
        m_goal = null;
        actuate(setpoints, feedForwardTorqueNm);
    }

    /**
     * Pass the setpoint directly to the mechanism's position controller.
     * For outboard control we only use the "next" setpoint.
     */
    private void actuate(SetpointsR1 setpoints, double torqueNm) {
        // setpoint must be updated so the profile can see it
        m_nextSetpoint = setpoints.next();
        double positionM = m_nextSetpoint.x();
        double velocityM_S = m_nextSetpoint.v();
        double accelM_S2 = m_nextSetpoint.a();
        m_mechanism.setPosition(
                positionM,
                velocityM_S,
                accelM_S2,
                torqueNm);
        m_log_control.log(() -> m_nextSetpoint);
        m_log_ff_torque.log(() -> torqueNm);
    }

    @Override
    public double getPosition() {
        return m_mechanism.getPositionM();
    }

    /** This is for wrist feedforard. */
    public double getSetpointAcceleration() {
        return m_nextSetpoint.a();
    }

    public void setDutyCycle(double value) {
        m_goal = null;
        m_nextSetpoint = null;
        m_mechanism.setDutyCycle(value);
    }

    @Override
    public double getVelocity() {
        return m_mechanism.getVelocityM_S();
    }

    @Override
    public boolean atSetpoint() {
        double pErr = m_nextSetpoint.x() - m_mechanism.getPositionM();
        double vErr = m_nextSetpoint.v() - m_mechanism.getVelocityM_S();
        return Math.abs(pErr) < m_positionTolerance
                && Math.abs(vErr) < m_velocityTolerance;
    }

    @Override
    public boolean profileDone() {
        if (m_goal == null) {
            // if there's no profile, it's always done.
            return true;
        }
        return m_ref.profileDone();
    }

    @Override
    public boolean atGoal() {
        return atSetpoint() && profileDone();
    }

    @Override
    public void stop() {
        m_mechanism.stop();
    }

    @Override
    public void close() {
        m_mechanism.close();
    }

    @Override
    public void periodic() {
        m_mechanism.periodic();
        m_log_position.log(() -> getPosition());
        m_log_velocity.log(() -> getVelocity());
    }
}
