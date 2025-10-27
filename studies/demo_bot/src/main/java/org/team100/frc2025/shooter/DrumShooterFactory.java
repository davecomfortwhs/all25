package org.team100.frc2025.shooter;

import org.team100.lib.examples.shooter.DualDrumShooter;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550Factory;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.util.CanId;

/** Configuration of motors on the demobot shooter. */
public class DrumShooterFactory {
    private static final double GEAR_RATIO = 5.2307692308;
    private static final double WHEEL_DIA_M = .33;

    public static DualDrumShooter make(LoggerFactory parent, int currentLimit) {
        LoggerFactory log = parent.name("shooter");
        LinearVelocityServo m_left = Neo550Factory.getNEO550VelocityServo(
                log.name("left"),
                currentLimit,
                new CanId(39),
                GEAR_RATIO,
                NeutralMode.BRAKE,
                MotorPhase.FORWARD,
                WHEEL_DIA_M);
        LinearVelocityServo m_right = Neo550Factory.getNEO550VelocityServo(
                log.name("right"),
                currentLimit,
                new CanId(19),
                GEAR_RATIO,
                NeutralMode.BRAKE,
                MotorPhase.REVERSE,
                WHEEL_DIA_M);

        return new DualDrumShooter(parent, m_left, m_right);
    }

}
