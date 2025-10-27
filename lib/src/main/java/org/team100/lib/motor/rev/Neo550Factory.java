package org.team100.lib.motor.rev;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.rev.CANSparkEncoder;
import org.team100.lib.encoder.sim.SimulatedBareEncoder;
import org.team100.lib.encoder.sim.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.util.CanId;

public class Neo550Factory {

    public static LinearMechanism getNEO550LinearMechanism(
            LoggerFactory log,
            int statorCurrentLimit,
            CanId canID,
            double gearRatio,
            NeutralMode neutral,
            MotorPhase motorPhase,
            double wheelDiameterM) {
        Feedforward100 ff = Feedforward100.makeNeo550(log);
        PIDConstants pid = PIDConstants.zero(log);
        Neo550CANSparkMotor motor = new Neo550CANSparkMotor(
                log,
                canID,
                neutral,
                motorPhase,
                statorCurrentLimit,
                ff,
                pid);
        CANSparkEncoder encoder = new CANSparkEncoder(log, motor);
        return new LinearMechanism(
                log, motor, encoder, gearRatio, wheelDiameterM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public static RotaryMechanism getNEO550RotaryMechanism(
            LoggerFactory log,
            int statorCurrentLimit,
            CanId canID,
            double gearRatio,
            NeutralMode neutral,
            MotorPhase motorPhase) {
        Feedforward100 ff = Feedforward100.makeNeo550(log);
        PIDConstants pid = PIDConstants.makePositionPID(log, 1);
        Neo550CANSparkMotor motor = new Neo550CANSparkMotor(
                log,
                canID,
                neutral,
                motorPhase,
                statorCurrentLimit,
                ff,
                pid);
        CANSparkEncoder encoder = new CANSparkEncoder(log, motor);
        ProxyRotaryPositionSensor sensor = new ProxyRotaryPositionSensor(encoder, gearRatio);
        return new RotaryMechanism(
                log,
                motor,
                sensor,
                gearRatio,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public static LinearVelocityServo getNEO550VelocityServo(
            LoggerFactory log,
            int statorCurrentLimit,
            CanId canID,
            double gearRatio,
            NeutralMode neutral,
            MotorPhase motorPhase,
            double wheelDiameterM) {
        return new OutboardLinearVelocityServo(log,
                getNEO550LinearMechanism(
                        log,
                        statorCurrentLimit,
                        canID,
                        gearRatio,
                        neutral,
                        motorPhase,
                        wheelDiameterM));
    }

    public static OutboardLinearVelocityServo simulatedDriveServo(LoggerFactory parent) {
        return new OutboardLinearVelocityServo(
                parent,
                simulatedLinearMechanism(parent));
    }

    public static RotaryMechanism simulatedRotaryMechanism(LoggerFactory parent) {
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(parent, driveMotor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(parent, encoder, 1);
        return new RotaryMechanism(
                parent, driveMotor, sensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public static LinearMechanism simulatedLinearMechanism(LoggerFactory parent) {
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(parent, driveMotor);
        return new LinearMechanism(
                parent,
                driveMotor, encoder, 1, 2, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }
}
