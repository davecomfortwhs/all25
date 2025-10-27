package org.team100.lib.config;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.tuning.Mutable;

/**
 * Be careful of the units for these parameters: different units will be used by
 * different outboard motor controllers, and by the same outboard motor
 * controller when used in different ways. Please add a comment about units to
 * every use of this class.
 */
public class PIDConstants {

    private final List<Runnable> m_listeners = new ArrayList<>();

    // private final double m_positionP;
    private final Mutable m_positionP;
    private final double m_positionI;
    private final double m_positionD;
    private final double m_positionIZone;
    // private final double m_velocityP;
    // try making velocity P not really a constant
    private final Mutable m_velocityP;
    private final double m_velocityI;
    private final double m_velocityD;
    private final double m_velocityIZone;

    public static PIDConstants zero(LoggerFactory log) {
        return new PIDConstants(log, 0, 0);
    }

    public static PIDConstants makeVelocityPID(LoggerFactory log, double p) {
        return new PIDConstants(log, 0, p);
    }

    public static PIDConstants makePositionPID(LoggerFactory log, double p) {
        return new PIDConstants(log, p, 0);
    }

    public double getPositionP() {
        return m_positionP.getAsDouble();
    }

    public double getPositionI() {
        return m_positionI;
    }

    public double getPositionD() {
        return m_positionD;
    }

    public double getPositionIZone() {
        return m_positionIZone;
    }

    public double getVelocityP() {
        return m_velocityP.getAsDouble();
    }

    public double getVelocityI() {
        return m_velocityI;
    }

    public double getVelocityD() {
        return m_velocityD;
    }

    public double getVelocityIZone() {
        return m_velocityIZone;
    }

    public void register(Runnable listener) {
        m_listeners.add(listener);
    }

    //////////////////////////////////////////////////////

    private PIDConstants(LoggerFactory log, double positionP, double velocityP) {
        this(log, positionP, 0, 0, velocityP, 0, 0);
    }

    private PIDConstants(LoggerFactory log, double positionP, double positionI, double positionD, double velocityP,
            double velocityI,
            double velocityD) {
        // m_positionP = positionP;
        m_positionP = new Mutable(log, "position P", positionP, this::onChange);
        m_positionI = positionI;
        m_positionD = positionD;
        m_positionIZone = 0;
        // m_velocityP = velocityP;
        m_velocityP = new Mutable(log, "velocity P", velocityP, this::onChange);
        m_velocityI = velocityI;
        m_velocityD = velocityD;
        m_velocityIZone = 0;
    }

    private void onChange(double ignored) {
        m_listeners.stream().forEach(r -> r.run());
    }
}
