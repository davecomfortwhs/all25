package org.team100.lib.encoder.wpi;

import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.util.RoboRioChannel;

/**
 * Implements max and min duty cycle to address the PWM output's "init" and
 * "exit" clock cycles.
 * 
 * 12 init cycles (always high)
 * 4 error cycles (always high)
 * 4096 data cycles
 * 8 exit cycles (always low)
 * 
 * The total number of cycles is 12+4+4095+8 = 4119.
 * 
 * The minimum value is (12+4) / 4119 or 0.003888
 * 
 * The maximum value is (12+4+4095) / 4119 or 0.998058
 * 
 * https://docs.google.com/document/d/1Znb4MQAqJWQ_Wk_SDJvLRaJxrAjNylkQKeC4OdLdMco/edit
 * 
 * The 5048 encoder (and, I think, all AMS sensors) produce clockwise-positive
 * output, looking down onto the top face of the sensor package. Our usual
 * mounting orientation puts this face against the rotating part, so our view of
 * the sensor is through the back. In this orientation, the output will seem to
 * be CCW-positive.
 */
public class AS5048RotaryPositionSensor extends DutyCycleRotaryPositionSensor {

    public AS5048RotaryPositionSensor(
            LoggerFactory parent,
            RoboRioChannel channel,
            double inputOffset,
            EncoderDrive drive) {
        super(parent, channel, inputOffset, drive);
    }

    protected double m_sensorMin() {
        return 0.003888;
    }

    protected double m_sensorMax() {
        return 0.998058;
    }
}
