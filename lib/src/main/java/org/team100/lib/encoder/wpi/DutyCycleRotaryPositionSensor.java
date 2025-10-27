package org.team100.lib.encoder.wpi;

import java.util.function.DoubleSupplier;

import org.team100.lib.coherence.Cache;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.IntLogger;
import org.team100.lib.util.RoboRioChannel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

/**
 * Absolute rotary position sensor using duty cycle input.
 * 
 * Duty cycle input is more robust to noise than the analog inputs, but not
 * totally immune: we saw issues in 2025 where high-frequency noise in the 5v
 * supply would produce noisy sensor output, and completely confuse the FPGA
 * counter.
 * 
 * Note that for the first few seconds after the sensor is constructed on the
 * RoboRIO, the duty cycle input produces garbage.  So the Robot class should sleep awhile.
 * 
 * Relies on Memo and Takt, so you must put Memo.resetAll() and Takt.update() in
 * Robot.robotPeriodic().
 */
public abstract class DutyCycleRotaryPositionSensor extends RoboRioRotaryPositionSensor {
    /**
     * Should sensor disconnect be a fatal error? I think in practice it is, but you
     * might want to turn this off during development.
     */
    private static final boolean THROW_IF_DISCONNNECTED = true;
    private static final int FREQ_THRESHOLD = 500;

    private final int m_channel;
    private final DigitalInput m_digitalInput;
    private final DutyCycle m_dutyCycle;
    private final DoubleSupplier m_duty;
    private final DoubleLogger m_log_duty;
    private final IntLogger m_log_frequency;
    private final BooleanLogger m_log_connected;

    protected DutyCycleRotaryPositionSensor(
            LoggerFactory parent,
            RoboRioChannel channel,
            double inputOffset,
            EncoderDrive drive) {
        super(parent, inputOffset, drive);
        LoggerFactory child = parent.type(this);
        m_channel = channel.channel;
        m_digitalInput = new DigitalInput(channel.channel);
        m_dutyCycle = new DutyCycle(m_digitalInput);
        m_duty = Cache.ofDouble(m_dutyCycle::getOutput);
        m_log_duty = child.doubleLogger(Level.COMP, "duty cycle");
        m_log_frequency = child.intLogger(Level.TRACE, "frequency");
        m_log_connected = child.booleanLogger(Level.TRACE, "connected");
        child.intLogger(Level.COMP, "channel").log(() -> channel.channel);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void close() {
        m_dutyCycle.close();
        m_digitalInput.close();
    }

    /**
     * If the encoder becomes disconnected, this will either return garbage or throw
     * IllegalStateException, depending on THROW_IF_DISCONNECTED.
     * 
     * Disconnects used to return Optional.empty, but it never happened in practice,
     * so I took it out to simplify the API.
     * 
     * Cached.
     */
    @Override
    protected double getRatio() {
        if (!isConnected()) {
            m_log_connected.log(() -> false);
            String msg = String.format("*** encoder %d not connected ***", m_channel);
            System.out.println("WARNING: " + msg);
            if (THROW_IF_DISCONNNECTED)
                throw new IllegalStateException(msg);
        }
        m_log_connected.log(() -> true);
        double dutyCycle = m_duty.getAsDouble();
        m_log_duty.log(() -> dutyCycle);
        return dutyCycle;
    }

    private boolean isConnected() {
        int frequency = m_dutyCycle.getFrequency();
        m_log_frequency.log(() -> frequency);
        return frequency > FREQ_THRESHOLD;
    }

    /** For testing */
    DutyCycle getDutyCycle() {
        return m_dutyCycle;
    }
}
