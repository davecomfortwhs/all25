package org.team100.ballerina;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class Indicator {
    private static final int LENGTH = 10;
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_greenBuffer;
    private final AddressableLEDBuffer m_redBuffer;
    private final BooleanSupplier m_onTarget;

    public Indicator(BooleanSupplier onTarget) {
        m_onTarget = onTarget;
        m_led = new AddressableLED(0);
        m_led.setLength(LENGTH);
        m_led.start();
        m_greenBuffer = fill(Color.kGreen);
        m_redBuffer = fill(Color.kRed);
    }

    public void periodic() {
        if (m_onTarget.getAsBoolean()) {
            m_led.setData(m_greenBuffer);
        } else {
            m_led.setData(m_redBuffer);
        }
    }

    public void close() {
        m_led.close();
    }

    private static AddressableLEDBuffer fill(Color color) {
        AddressableLEDBuffer buf = new AddressableLEDBuffer(LENGTH);
        for (int i = 0; i < LENGTH; ++i) {
            buf.setLED(i, color);
        }
        return buf;
    }

}
