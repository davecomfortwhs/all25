package org.team100.lib.encoder.ctre;

import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.ctre.Talon6Motor;

public class Talon6Encoder implements IncrementalBareEncoder {
    private final Talon6Motor m_motor;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;

    public Talon6Encoder(LoggerFactory parent, Talon6Motor motor) {
        LoggerFactory child = parent.type(this);
        m_motor = motor;
        m_log_position = child.doubleLogger(Level.TRACE, "position (rad)");
        m_log_velocity = child.doubleLogger(Level.TRACE, "velocity (rad_s)");
        reset();
    }

    /**
     * Not latency-compensated.
     * Value is updated in Robot.robotPeriodic().
     */
    @Override
    public double getVelocityRad_S() {
        return m_motor.getVelocityRev_S() * 2.0 * Math.PI;
    }

    /**
     * Latency-compensated.
     * Value is updated in Robot.robotPeriodic().
     */
    @Override
    public double getUnwrappedPositionRad() {
        return m_motor.getUnwrappedPositionRad();
    }

    @Override
    public void reset() {
        m_motor.resetEncoderPosition();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    /**
     * Set integrated sensor position in radians.
     * 
     * Note this takes **FOREVER**, like tens of milliseconds, so you can only do it
     * at startup.
     */
    @Override
    public void setUnwrappedEncoderPositionRad(double motorPositionRad) {
        m_motor.setUnwrappedEncoderPositionRad(motorPositionRad);
    }

    @Override
    public void periodic() {
        m_log_position.log(this::getUnwrappedPositionRad);
        m_log_velocity.log(this::getVelocityRad_S);
    }

}
