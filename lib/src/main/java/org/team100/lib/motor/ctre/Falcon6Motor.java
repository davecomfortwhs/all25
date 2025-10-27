package org.team100.lib.motor.ctre;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.util.CanId;

/**
 * Falcon 500 using Phoenix 6.
 * 
 * @see https://store.ctr-electronics.com/content/datasheet/Motor%20Performance%20Analysis%20Report.pdf
 */
public class Falcon6Motor extends Talon6Motor {

    public Falcon6Motor(
            LoggerFactory parent,
            CanId canId,
            NeutralMode neutral,
            MotorPhase motorPhase,
            double supplyLimit,
            double statorLimit,
            PIDConstants pid,
            Feedforward100 ff) {
        super(parent, canId, neutral, motorPhase, supplyLimit, statorLimit, pid, ff);
    }

    @Override
    public double kROhms() {
        return 0.03;
    }

    @Override
    public double kTNm_amp() {
        return 0.018;
    }
}
