package org.team100.lib.motion.mechanism;

import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.music.Player;
import org.team100.lib.state.Model100;

/**
 * Uses a motor and gears to produce rotational output, e.g. an arm joint.
 * 
 * Motor velocity and accel is higher than mechanism, required torque is lower,
 * using the supplied gear ratio.
 * 
 * The position limits used to be enforced by a proxy, but now they're here: it
 * seems simpler that way.
 */
public class RotaryMechanism implements Player {
    private final BareMotor m_motor;
    private final RotaryPositionSensor m_sensor;
    private final double m_gearRatio;
    private final double m_minPositionRad;
    private final double m_maxPositionRad;
    private final DoubleLogger m_log_velocity;
    private final DoubleLogger m_log_position;

    /**
     * The provided sensor encapsulates the motor sensor and/or the external
     * absolute sensor, if used. See ProxyRotaryPositionSensor and
     * CombinedRotaryPositionSensor.
     */
    public RotaryMechanism(
            LoggerFactory parent,
            BareMotor motor,
            RotaryPositionSensor sensor,
            double gearRatio,
            double minPositionRad,
            double maxPositionRad) {
        LoggerFactory child = parent.type(this);
        m_motor = motor;
        m_sensor = sensor;
        m_gearRatio = gearRatio;
        m_minPositionRad = minPositionRad;
        m_maxPositionRad = maxPositionRad;
        m_log_velocity = child.doubleLogger(Level.DEBUG, "velocity (rad_s)");
        m_log_position = child.doubleLogger(Level.DEBUG, "position (rad)");
    }

    /** Use for homing. */
    public void setDutyCycleUnlimited(double output) {
        m_motor.setDutyCycle(output);
    }

    /** Should actuate immediately. Enforces position limit using the encoder. */
    public void setDutyCycle(double output) {
        double posRad = getWrappedPositionRad();
        if (output < 0 && posRad < m_minPositionRad) {
            m_motor.stop();
            return;
        }
        if (output > 0 && posRad > m_maxPositionRad) {
            m_motor.stop();
            return;
        }
        m_motor.setDutyCycle(output);
    }

    public void setTorqueLimit(double torqueNm) {
        m_motor.setTorqueLimit(torqueNm / m_gearRatio);
    }

    /** Should actuate immediately. Use for homing. */
    public void setVelocityUnlimited(
            double outputRad_S,
            double outputAccelRad_S2,
            double outputTorqueNm) {
        m_motor.setVelocity(
                outputRad_S * m_gearRatio,
                outputAccelRad_S2 * m_gearRatio,
                outputTorqueNm / m_gearRatio);
    }

    /** Should actuate immediately. Enforces position limit using the encoder. */
    public void setVelocity(
            double velocityRad_S,
            double accelRad_S2,
            double torqueNm) {
        double posRad = getWrappedPositionRad();
        if (velocityRad_S < 0 && posRad < m_minPositionRad) {
            m_motor.stop();
            return;
        }
        if (velocityRad_S > 0 && posRad > m_maxPositionRad) {
            m_motor.stop();
            return;
        }
        m_motor.setVelocity(
                velocityRad_S * m_gearRatio,
                accelRad_S2 * m_gearRatio,
                torqueNm / m_gearRatio);
    }

    /**
     * Apply limits and gear ratio, and set the resulting motor position.
     * 
     * This is the "unwrapped" position, i.e. the domain is infinite, not cyclical
     * within +/- pi.
     * 
     * Should actuate immediately.
     * 
     * Make sure you don't double-count factors of torque/accel.
     */
    public void setUnwrappedPosition(
            double positionRad,
            double velocityRad_S,
            double accelRad_S2,
            double torqueNm) {
        if (positionRad < m_minPositionRad) {
            System.out.printf("WARNING: requested position %8.3f less than min %8.3f\n",
                    positionRad, m_minPositionRad);
            m_motor.stop();
            return;
        }
        if (positionRad > m_maxPositionRad) {
            System.out.printf("WARNING: requested position %8.3f more than max %8.3f\n",
                    positionRad, m_maxPositionRad);
            m_motor.stop();
            return;
        }
        m_motor.setUnwrappedPosition(
                positionRad * m_gearRatio,
                velocityRad_S * m_gearRatio,
                accelRad_S2 * m_gearRatio,
                torqueNm / m_gearRatio);
    }

    public Model100 getUnwrappedMeasurement() {
        return new Model100(getUnwrappedPositionRad(), getVelocityRad_S());
    }

    /**
     * Value is updated in Robot.robotPeriodic().
     * 
     * @return velocity in rad/s
     */
    public double getVelocityRad_S() {
        return m_sensor.getVelocityRad_S();
    }

    /**
     * Returns the "wrapped" angular position, i.e. this dimension is cyclical, with
     * values beyond +/- pi mapped back to the +/- pi interval: 2pi is mapped to 0,
     * 5pi/4 is mapped to pi/4, etc.
     * 
     * @return the absolute 1:1 position of the mechanism in rad [-pi, pi]
     */
    public double getWrappedPositionRad() {
        return m_sensor.getWrappedPositionRad();
    }

    /** Unwrapped domain is infinite. */
    public double getUnwrappedPositionRad() {
        return m_sensor.getUnwrappedPositionRad();
    }

    /** Minimum unwrapped position. */
    public double getMinPositionRad() {
        return m_minPositionRad;
    }

    /** Maximum unwrapped position. */
    public double getMaxPositionRad() {
        return m_maxPositionRad;
    }

    public void stop() {
        m_motor.stop();
    }

    public void close() {
        m_motor.close();
    }

    public void periodic() {
        m_motor.periodic();
        m_sensor.periodic();
        m_log_velocity.log(() -> getVelocityRad_S());
        m_log_position.log(() -> getWrappedPositionRad());
    }

    @Override
    public void play(double freq) {
        m_motor.play(freq);
    }

}
