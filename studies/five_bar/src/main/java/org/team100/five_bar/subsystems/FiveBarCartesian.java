package org.team100.five_bar.subsystems;

import java.util.function.Supplier;

import org.team100.five_bar.commands.Move;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.five_bar.ActuatorAngles;
import org.team100.lib.motion.five_bar.FiveBarKinematics;
import org.team100.lib.motion.five_bar.JointPositions;
import org.team100.lib.motion.five_bar.Scenario;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.util.CanId;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Cartesian control using inverse kinematics, and without profiling.
 */
public class FiveBarCartesian extends SubsystemBase {
    /** Low current limits */
    private static final double SUPPLY_LIMIT = 5;
    private static final double STATOR_LIMIT = 5;
    private static final Scenario SCENARIO;
    static {
        // origin is P1
        SCENARIO = new Scenario();
        // TODO: real measurements
        SCENARIO.a1 = 0.1;
        SCENARIO.a2 = 0.1;
        SCENARIO.a3 = 0.1;
        SCENARIO.a4 = 0.1;
        SCENARIO.a5 = 0.1;
        SCENARIO.xcenter = 0.5;
        SCENARIO.ycenter = 0.15;
    }

    /** Left motor, "P1" in the diagram. */
    /**
     * There's no absolute encoder in the apparatus, so we use a "proxy" instead;
     * this needs a "homing" mechanism of some kind.
     */
    private final ProxyRotaryPositionSensor m_sensorP1;
    private final RotaryMechanism m_mechP1;

    /** Right motor, "P5" in the diagram. */
    private final ProxyRotaryPositionSensor m_sensorP5;
    private final RotaryMechanism m_mechP5;

    public FiveBarCartesian(LoggerFactory logger) {
        // zeros
        PIDConstants pid = new PIDConstants();
        Feedforward100 ff = Feedforward100.zero();

        LoggerFactory loggerP1 = logger.name("p1");
        Falcon6Motor motorP1 = new Falcon6Motor(
                loggerP1,
                new CanId(1),
                NeutralMode.COAST,
                MotorPhase.FORWARD,
                SUPPLY_LIMIT,
                STATOR_LIMIT,
                pid,
                ff);
        IncrementalBareEncoder encoderP1 = new Talon6Encoder(loggerP1, motorP1);
        m_sensorP1 = new ProxyRotaryPositionSensor(encoderP1, 1.0);
        m_mechP1 = new RotaryMechanism(
                loggerP1,
                motorP1,
                m_sensorP1,
                1.0,
                0.0,
                1.0);

        LoggerFactory loggerP5 = logger.name("p5");
        Falcon6Motor motorP5 = new Falcon6Motor(
                loggerP5,
                new CanId(2),
                NeutralMode.COAST,
                MotorPhase.FORWARD,
                SUPPLY_LIMIT,
                STATOR_LIMIT,
                pid,
                ff);
        IncrementalBareEncoder encoderP5 = new Talon6Encoder(loggerP5, motorP5);
        m_sensorP5 = new ProxyRotaryPositionSensor(encoderP5, 1.0);
        m_mechP5 = new RotaryMechanism(
                loggerP5,
                motorP5,
                m_sensorP5,
                1.0,
                0.0,
                1.0);
    }

    /**
     * Moves both axes so that P3 reaches the specified translation relative to the
     * work center. Movement is uncoordinated, so the caller should manage the
     * trajectory, if desired.
     * 
     * TODO: velocity feedforward
     */
    public void setPosition(Translation2d t) {
        double x3 = t.getX();
        double y3 = t.getY();
        ActuatorAngles p = FiveBarKinematics.inverse(
                SCENARIO, x3 + SCENARIO.xcenter, y3 + SCENARIO.ycenter);
        m_mechP1.setUnwrappedPosition(p.q1(), 0, 0, 0);
        m_mechP5.setUnwrappedPosition(p.q5(), 0, 0, 0);
    }

    public JointPositions getJointPositions() {
        double q1 = m_mechP1.getWrappedPositionRad();
        double q5 = m_mechP5.getWrappedPositionRad();
        return FiveBarKinematics.forward(SCENARIO, q1, q5);
    }

    public Translation2d getPosition() {
        double q1 = m_mechP1.getWrappedPositionRad();
        double q5 = m_mechP5.getWrappedPositionRad();
        JointPositions j = FiveBarKinematics.forward(SCENARIO, q1, q5);
        return new Translation2d(j.P3().x(), j.P3().y());
    }

    public Command move(Translation2d goal) {
        Move m = new Move(this, goal, 0.1);
        return m.until(m::done);
    }

    @Override
    public void periodic() {
        m_mechP1.periodic();
        m_mechP5.periodic();
    }

    //////////////////////

    private void setDutyCycle(double p1, double p5) {
        m_mechP1.setDutyCycle(p1);
        m_mechP5.setDutyCycle(p5);
    }

    private void resetEncoderPosition() {
        m_sensorP1.setEncoderPosition(0);
        m_sensorP5.setEncoderPosition(0);
    }

    ///////////////////////
    //
    // Commands

    public Command home() {
        return run(() -> setDutyCycle(0.05, 0.05));
    }

    public Command zero() {
        return runOnce(this::resetEncoderPosition);
    }

    public Command position(Supplier<Translation2d> t) {
        return run(() -> setPosition(t.get()));
    }
}
