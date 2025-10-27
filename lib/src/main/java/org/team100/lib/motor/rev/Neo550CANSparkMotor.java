package org.team100.lib.motor.rev;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.util.CanId;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/**
 * Neo550 motor.
 * 
 * @see https://www.revrobotics.com/rev-21-1651/
 */
public class Neo550CANSparkMotor extends CANSparkMotor {
    /**
     * Current limit is stator current.
     */
    public Neo550CANSparkMotor(
            LoggerFactory parent,
            CanId canId,
            NeutralMode neutral,
            MotorPhase motorPhase,
            int statorCurrentLimit,
            Feedforward100 ff,
            PIDConstants pid) {
        super(parent, new SparkMax(canId.id, MotorType.kBrushless),
                neutral, motorPhase, statorCurrentLimit, ff, pid);
    }

    @Override
    public double kROhms() {
        return 0.12;
    }

    @Override
    public double kTNm_amp() {
        return 0.009;
    }
}
