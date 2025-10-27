package org.team100.frc2025.shooter;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550CANSparkMotor;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {

    private final Neo550CANSparkMotor m_pivot;
    private final DoubleLogger m_log_angle;

    public PivotSubsystem(LoggerFactory parent, int currentLimit) {
        LoggerFactory logger = parent.type(this);
        m_log_angle = logger.doubleLogger(Level.TRACE, "Angle (rad)");
        m_pivot = new Neo550CANSparkMotor(
                logger,
                new CanId(5),
                NeutralMode.BRAKE,
                MotorPhase.FORWARD,
                currentLimit,
                Feedforward100.makeNeo550(),
                new PIDConstants());
    }

    public void dutyCycle(double set) {
        m_pivot.setDutyCycle(set);
    }

    public double getAngleRad() {
        return m_pivot.getPositionRot();
    }

    public void setEncoderPosition(double positionRad) {
        m_pivot.setEncoderPosition(positionRad);
    }

    public void setTorqueLimit(double value) {
        m_pivot.setTorqueLimit(value);
    }

    public void stop() {
        m_pivot.stop();
    }

    @Override
    public void periodic() {
        m_pivot.periodic();
        m_log_angle.log(this::getAngleRad);
    }
}
