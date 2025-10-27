package org.team100.frc2025;

import org.team100.lib.examples.tank.TankDrive;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanismFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.util.CanId;

/** Configuration of motors on the Demo Bot. */
public class TankFactory {
    private static final double FIVE_TO_ONE = 5.2307692308;
    private static final double GEAR_RATIO = FIVE_TO_ONE * FIVE_TO_ONE;
    private static final double WHEEL_DIAM = 0.098425;

    public static TankDrive make(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            int supplyLimit) {
        LoggerFactory log = parent.type("Tank Drive");
        return new TankDrive(fieldLogger,
                LinearMechanismFactory.neo550(
                        log.name("left"), supplyLimit, new CanId(3),
                        MotorPhase.REVERSE, GEAR_RATIO, WHEEL_DIAM),
                LinearMechanismFactory.neo550(
                        log.name("left"), supplyLimit, new CanId(2),
                        MotorPhase.FORWARD, GEAR_RATIO, WHEEL_DIAM));
    }
}
