package org.team100.lib.motor.ctre;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.util.CanId;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/** Any motor connected to the Talon SRX controller. */
public class TalonSRXMotor implements BareMotor {
    private static final double FF_DUTY_RAD_S = 0.0016;

    private final TalonSRX m_motor;
    private final DoubleLogger m_log_supply;
    private final DoubleLogger m_log_stator;
    private final DoubleLogger m_log_duty;

    public TalonSRXMotor(
            LoggerFactory parent,
            CanId canID,
            MotorPhase phase,
            NeutralMode neutral,
            double supplyLimit) {
        m_motor = new TalonSRX(canID.id);
        switch (neutral) {
            case COAST -> m_motor.setNeutralMode(
                    com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
            case BRAKE -> m_motor.setNeutralMode(
                    com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
        }
        switch (phase) {
            case FORWARD -> m_motor.setInverted(false);
            case REVERSE -> m_motor.setInverted(true);
        }
        // don't use the "peak" current limit feature at all
        m_motor.configPeakCurrentLimit(0);
        // the supply limit is really an input power limit; the available torque thus
        // varies with RPM.
        m_motor.configContinuousCurrentLimit((int) supplyLimit);
        m_motor.enableCurrentLimit(true);
        LoggerFactory log = parent.type(this);
        m_log_supply = log.doubleLogger(Level.TRACE, "supply current (A)");
        m_log_stator = log.doubleLogger(Level.TRACE, "stator current (A)");
        m_log_duty = log.doubleLogger(Level.TRACE, "duty cycle");
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    public void setVelocity(double velocityRad_S, double accelRad_S2, double torqueNm) {
        final double motorDutyCycle = velocityRad_S * FF_DUTY_RAD_S;
        setDutyCycle(motorDutyCycle);
    }

    @Override
    public double kROhms() {
        // this is the number for a CIM; if you use this for any other motor, you should
        // adjust it.
        return 0.09;
    }

    @Override
    public double kTNm_amp() {
        // this is the number for a CIM; if you use this for any other motor, you should
        // adjust it.
        return 0.018;
    }

    @Override
    public void stop() {
        m_motor.neutralOutput();
    }

    @Override
    public void reset() {
        //
    }

    @Override
    public void close() {
        // SRX doesn't support close()
    }

    @Override
    public void periodic() {
        m_log_supply.log(m_motor::getSupplyCurrent);
        m_log_stator.log(m_motor::getStatorCurrent);
        m_log_duty.log(m_motor::getMotorOutputPercent);
    }

    @Override
    public double getCurrent() {
        return m_motor.getStatorCurrent();
    }

    // unsupported methods

    @Override
    public void setTorqueLimit(double torqueNm) {
        throw new UnsupportedOperationException("TalonSRX limits power, not torque.");
    }

    @Override
    public double getVelocityRad_S() {
        throw new UnsupportedOperationException("TalonSRX cannot report velocity.");
    }

    @Override
    public double getUnwrappedPositionRad() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        // there actually is an encoder interface but it sucks
        throw new UnsupportedOperationException("TalonSRX has no encoder");
    }

    @Override
    public void setUnwrappedPosition(double positionRad, double velocityRad_S, double accelRad_S2, double torqueNm) {
        throw new UnsupportedOperationException("TalonSRX cannot control position.");
    }

    @Override
    public void play(double freq) {
    }

}
