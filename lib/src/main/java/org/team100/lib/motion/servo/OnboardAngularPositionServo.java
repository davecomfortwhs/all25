package org.team100.lib.motion.servo;

import org.team100.lib.controller.r1.Feedback100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Model100Logger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.SetpointsR1;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * Uses mechanism velocity control.
 * 
 * Uses a profile with velocity feedforward, feedback here in Java-land, and
 * extra torque (e.g. for gravity).
 */
public class OnboardAngularPositionServo extends AngularPositionServoImpl {
    private static final boolean DEBUG = false;

    private final Feedback100 m_feedback;

    private final DoubleLogger m_log_feedforward_torque;
    private final Model100Logger m_log_measurement;
    private final Control100Logger m_log_control;
    private final DoubleLogger m_log_u_FB;
    private final DoubleLogger m_log_u_FF;
    private final DoubleLogger m_log_u_TOTAL;
    private final DoubleLogger m_log_error;
    private final DoubleLogger m_log_velocity_error;

    public OnboardAngularPositionServo(
            LoggerFactory parent,
            RotaryMechanism mech,
            ProfileReferenceR1 ref,
            Feedback100 feedback) {
        super(parent, mech, ref);
        if (feedback.handlesWrapping())
            throw new IllegalArgumentException("Do not supply wrapping feedback");
        LoggerFactory child = parent.type(this);
        m_feedback = feedback;

        m_log_feedforward_torque = child.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_measurement = child.model100Logger(Level.COMP, "measurement (rad)");
        m_log_control = child.control100Logger(Level.COMP, "control (rad)");
        m_log_u_FB = child.doubleLogger(Level.TRACE, "u_FB (rad_s)");
        m_log_u_FF = child.doubleLogger(Level.TRACE, "u_FF (rad_s)");

        m_log_u_TOTAL = child.doubleLogger(Level.COMP, "u_TOTAL (rad_s)");
        m_log_error = child.doubleLogger(Level.TRACE, "Controller Position Error (rad)");
        m_log_velocity_error = child.doubleLogger(Level.TRACE, "Controller Velocity Error (rad_s)");
    }

    @Override
    public void reset() {
        super.reset();
        m_feedback.reset();
    }

    /**
     * Feedback using measurement and current setpoint. Feedforward using next
     * setpoint.
     */
    void actuate(SetpointsR1 unwrappedSetpoint, double feedForwardTorqueNm) {
        if (DEBUG) {
            System.out.printf("setpoint %s\n", unwrappedSetpoint);
        }

        Model100 unwrappedMeasurement = m_mechanism.getUnwrappedMeasurement();
        Model100 currentUnwrappedSetpoint = unwrappedSetpoint.current().model();
        Control100 nextUnwrappedSetpoint = unwrappedSetpoint.next();

        if (DEBUG) {
            System.out.printf("unwrapped Measurement %s currentUnwrappedSetpoint %s\n",
                    unwrappedMeasurement, currentUnwrappedSetpoint);
        }
        double u_FB = m_feedback.calculate(unwrappedMeasurement, currentUnwrappedSetpoint);
        double u_FF = nextUnwrappedSetpoint.v();
        double u_TOTAL = u_FB + u_FF;
        if (DEBUG) {
            System.out.printf("u_FB %6.3f u_FF %6.3f u_TOTAL %6.3f\n", u_FB, u_FF, u_TOTAL);
        }

        m_mechanism.setVelocity(u_TOTAL, nextUnwrappedSetpoint.a(), feedForwardTorqueNm);

        m_log_feedforward_torque.log(() -> feedForwardTorqueNm);
        m_log_measurement.log(() -> unwrappedMeasurement);
        m_log_control.log(() -> nextUnwrappedSetpoint);
        m_log_u_FB.log(() -> u_FB);
        m_log_u_FF.log(() -> u_FF);
        m_log_u_TOTAL.log(() -> u_TOTAL);
        m_log_error.log(() -> currentUnwrappedSetpoint.x() - unwrappedMeasurement.x());
        m_log_velocity_error.log(() -> currentUnwrappedSetpoint.v() - unwrappedMeasurement.v());
    }

}
