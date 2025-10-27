package org.team100.lib.motion.tank;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanismFactory;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.util.CanId;

public class TankDriveFactory {

    public static TankDrive make(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            int statorCurrentLimit,
            CanId left,
            CanId right,
            double trackWidthM,
            double gearRatio,
            double wheelDiaM) {
        return switch (Identity.instance) {
            case BLANK -> sim(
                    fieldLogger,
                    parent,
                    trackWidthM,
                    gearRatio,
                    wheelDiaM);
            default -> realRobot(
                    fieldLogger,
                    parent,
                    statorCurrentLimit,
                    left,
                    right,
                    trackWidthM,
                    gearRatio,
                    wheelDiaM);
        };
    }

    private static TankDrive realRobot(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            int statorCurrentLimit,
            CanId left,
            CanId right,
            double trackWidthM,
            double gearRatio,
            double wheelDiaM) {
        LoggerFactory log = parent.name("Tank Drive");
        LoggerFactory leftLog = log.name("left");
        LoggerFactory rightLog = log.name("right");
        return new TankDrive(
                fieldLogger,
                trackWidthM,
                new OutboardLinearVelocityServo(
                        leftLog,
                        LinearMechanismFactory.neo(
                                leftLog,
                                statorCurrentLimit,
                                left,
                                MotorPhase.REVERSE,
                                gearRatio,
                                wheelDiaM)),
                new OutboardLinearVelocityServo(
                        rightLog,
                        LinearMechanismFactory.neo(
                                rightLog,
                                statorCurrentLimit,
                                right,
                                MotorPhase.FORWARD,
                                gearRatio,
                                wheelDiaM)));
    }

    private static TankDrive sim(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            double trackWidthM,
            double gearRatio,
            double wheelDiaM) {
        LoggerFactory log = parent.name("Tank Drive");
        LoggerFactory leftLog = log.name("left");
        LoggerFactory rightLog = log.name("right");
        return new TankDrive(
                fieldLogger,
                trackWidthM,
                new OutboardLinearVelocityServo(
                        leftLog,
                        LinearMechanismFactory.sim(
                                leftLog,
                                gearRatio,
                                wheelDiaM)),
                new OutboardLinearVelocityServo(
                        rightLog,
                        LinearMechanismFactory.sim(
                                rightLog,
                                gearRatio,
                                wheelDiaM)));
    }
}
