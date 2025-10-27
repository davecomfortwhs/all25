package org.team100.lib.motion.mechanism;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.rev.CANSparkEncoder;
import org.team100.lib.encoder.sim.SimulatedBareEncoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.rev.Neo550CANSparkMotor;
import org.team100.lib.motor.rev.NeoCANSparkMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.util.CanId;

/**
 * Factory for common mechanisms.
 */
public class LinearMechanismFactory {

    /** REV NEO motor with built-in encoder. */
    public static LinearMechanism neo(
            LoggerFactory log,
            int statorCurrentLimit,
            CanId id,
            MotorPhase phase,
            double gearRatio,
            double wheelDiaM) {
        Feedforward100 ff = Feedforward100.makeNeo(log);
        // 10/22/25: Anay found this value worked well
        PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.00005);
        // 10/22/25: Lincoln used this value
        // PIDConstants.makeVelocityPID(0.0003));
        NeoCANSparkMotor left = new NeoCANSparkMotor(
                log, id, NeutralMode.BRAKE,
                phase, statorCurrentLimit,
                ff, pid);
        CANSparkEncoder leftEncoder = new CANSparkEncoder(
                log, left);
        LinearMechanism leftMech = new LinearMechanism(
                log, left, leftEncoder,
                gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        return leftMech;
    }

    /** REV NEO 550 motor with built-in encoder. */
    public static LinearMechanism neo550(
            LoggerFactory log,
            int statorCurrentLimit,
            CanId id,
            MotorPhase phase,
            double gearRatio,
            double wheelDiaM) {
        Feedforward100 ff = Feedforward100.makeNeo550(log);
        PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.0001);
        Neo550CANSparkMotor left = new Neo550CANSparkMotor(
                log, id, NeutralMode.BRAKE,
                phase, statorCurrentLimit,
                ff,
                pid);
        CANSparkEncoder leftEncoder = new CANSparkEncoder(log, left);
        LinearMechanism leftMech = new LinearMechanism(
                log, left, leftEncoder,
                gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        return leftMech;
    }

    /** Simulation. */
    public static LinearMechanism sim(
            LoggerFactory log,
            double gearRatio,
            double wheelDiaM) {
        BareMotor motor = new SimulatedBareMotor(log, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(
                log, motor);
        return new LinearMechanism(
                log, motor, encoder,
                gearRatio, wheelDiaM,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
    }
}
