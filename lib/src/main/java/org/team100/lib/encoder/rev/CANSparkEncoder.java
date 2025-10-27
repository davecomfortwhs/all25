package org.team100.lib.encoder.rev;

import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.rev.CANSparkMotor;

/**
 * The built-in encoder in Neo motors.
 * 
 * This encoder simply senses the 14 rotor magnets in 3 places, so it's 42 ticks
 * per turn.
 */
public class CANSparkEncoder implements IncrementalBareEncoder {
    private final CANSparkMotor m_motor;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;

    public CANSparkEncoder(LoggerFactory parent, CANSparkMotor motor) {
        LoggerFactory child = parent.type(this);
        m_motor = motor;
        m_log_position = child.doubleLogger(Level.TRACE, "position (rad)");
        m_log_velocity = child.doubleLogger(Level.TRACE, "velocity (rad_s)");
    }

    @Override
    public void reset() {
        m_motor.resetEncoderPosition();
    }

    @Override
    public void close() {
        //
    }

    //////////////////////////////////

    /**
     * Not latency-compensated.
     * Value is updated in Robot.robotPeriodic().
     */
    @Override
    public double getUnwrappedPositionRad() {
        return m_motor.getUnwrappedPositionRad();
    }

    /**
     * Not latency-compensated.
     * Value is updated in Robot.robotPeriodic().
     */
    @Override
    public double getVelocityRad_S() {
        // raw velocity is in RPM
        // this is fast so we don't need to cache it
        return m_motor.getRateRPM() * 2 * Math.PI / 60;
    }

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
