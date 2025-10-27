package org.team100.lib.motion.servo;

import java.util.function.DoubleUnaryOperator;

/** Represents extra torque for gravity and/or spring compensation. */
public class Torque {
    final DoubleUnaryOperator m_torque;

    public Torque(DoubleUnaryOperator torque) {
        m_torque = torque;
    }

    /**
     * I know it's weird to use an optional argument here but it keeps the default
     * in one place.
     */
    public double torque(double positionRad) {
        return m_torque.applyAsDouble(positionRad);
    }

}
