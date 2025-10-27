package org.team100.lib.motion.mecanum;

import org.team100.lib.config.Identity;
import org.team100.lib.gyro.Gyro;
import org.team100.lib.gyro.ReduxGyro;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mecanum.MecanumKinematics100.Slip;
import org.team100.lib.motion.mechanism.LinearMechanismFactory;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.util.CanId;

public class MecanumDriveFactory {

    public static MecanumDrive100 make(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            int statorCurrentLimit,
            CanId gyroId,
            CanId frontLeft,
            CanId frontRight,
            CanId rearLeft,
            CanId rearRight,
            double trackWidthM,
            double wheelbaseM,
            Slip slip,
            double gearRatio,
            double wheelDiaM) {
        return switch (Identity.instance) {
            case BLANK -> sim(
                    fieldLogger,
                    parent,
                    trackWidthM,
                    wheelbaseM,
                    gearRatio,
                    wheelDiaM);
            default -> realRobot(
                    fieldLogger, parent, statorCurrentLimit,
                    gyroId,
                    frontLeft,
                    frontRight,
                    rearLeft,
                    rearRight,
                    trackWidthM,
                    wheelbaseM,
                    slip,
                    gearRatio,
                    wheelDiaM);
        };
    }

    private static MecanumDrive100 realRobot(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            int statorCurrentLimit,
            CanId gyroId,
            CanId frontLeft,
            CanId frontRight,
            CanId rearLeft,
            CanId rearRight,
            double trackWidthM,
            double wheelbaseM,
            Slip slip,
            double gearRatio,
            double wheelDiaM) {
        LoggerFactory log = parent.name("Mecanum Drive");

        // null gyro => use odometry for yaw.
        Gyro gyro = (gyroId == null) ? null : new ReduxGyro(log, gyroId);

        LoggerFactory frontLeftLog = log.name("frontLeft");
        LoggerFactory frontRightLog = log.name("frontRight");
        LoggerFactory rearLeftLog = log.name("rearLeft");
        LoggerFactory rearRightLog = log.name("rearRight");
        return new MecanumDrive100(
                log,
                fieldLogger,
                gyro,
                trackWidthM,
                wheelbaseM,
                slip,
                new OutboardLinearVelocityServo(
                        frontLeftLog,
                        LinearMechanismFactory.neo(
                                frontLeftLog,
                                statorCurrentLimit,
                                frontLeft,
                                MotorPhase.REVERSE,
                                gearRatio,
                                wheelDiaM)),
                new OutboardLinearVelocityServo(
                        frontRightLog,
                        LinearMechanismFactory.neo(
                                frontRightLog,
                                statorCurrentLimit,
                                frontRight,
                                MotorPhase.FORWARD,
                                gearRatio,
                                wheelDiaM)),
                new OutboardLinearVelocityServo(
                        rearLeftLog,
                        LinearMechanismFactory.neo(
                                rearLeftLog,
                                statorCurrentLimit,
                                rearLeft,
                                MotorPhase.REVERSE,
                                gearRatio,
                                wheelDiaM)),
                new OutboardLinearVelocityServo(
                        rearRightLog,
                        LinearMechanismFactory.neo(
                                rearRightLog,
                                statorCurrentLimit,
                                rearRight,
                                MotorPhase.FORWARD,
                                gearRatio,
                                wheelDiaM)));
    }

    private static MecanumDrive100 sim(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            double trackWidthM,
            double wheelbaseM,
            double gearRatio,
            double wheelDiaM) {
        LoggerFactory log = parent.name("Mecanum Drive");
        LoggerFactory frontLeftLog = log.name("frontLeft");
        LoggerFactory frontRightLog = log.name("frontRight");
        LoggerFactory rearLeftLog = log.name("rearLeft");
        LoggerFactory rearRightLog = log.name("rearRight");

        // null gyro => use odometry for yaw
        return new MecanumDrive100(
                log,
                fieldLogger,
                null,
                trackWidthM,
                wheelbaseM,
                new Slip(1, 1, 1),
                new OutboardLinearVelocityServo(
                        frontLeftLog,
                        LinearMechanismFactory.sim(
                                frontLeftLog,
                                gearRatio,
                                wheelDiaM)),
                new OutboardLinearVelocityServo(
                        frontRightLog,
                        LinearMechanismFactory.sim(
                                frontRightLog,
                                gearRatio,
                                wheelDiaM)),
                new OutboardLinearVelocityServo(
                        rearLeftLog,
                        LinearMechanismFactory.sim(
                                rearLeftLog,
                                gearRatio,
                                wheelDiaM)),
                new OutboardLinearVelocityServo(
                        rearRightLog,
                        LinearMechanismFactory.sim(
                                rearRightLog,
                                gearRatio,
                                wheelDiaM)));
    }

}
