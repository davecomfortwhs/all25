package org.team100.lib.motor.rev;

import java.util.function.Supplier;

import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.tuning.Mutable;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Configuration for one SparkBase motor */
public class RevConfigurator {
    /**
     * The CAN report period for the built-in encoder.
     * This is longer than the equivalent CTRE number, maybe
     * it should be 10?
     */
    private static final int ENCODER_REPORT_PERIOD_MS = 20;

    private final SparkBase m_motor;
    private final NeutralMode m_neutral;
    private final MotorPhase m_phase;
    private final Mutable m_statorCurrentLimit;
    private final PIDConstants m_pid;

    /**
     * statorCurrentLimit is mutable.
     * pid has mutable parts.
     */
    public RevConfigurator(
            LoggerFactory log,
            SparkBase motor,
            NeutralMode neutral,
            MotorPhase phase,
            int statorCurrentLimit,
            PIDConstants pid) {
        m_motor = motor;
        m_neutral = neutral;
        m_phase = phase;
        m_statorCurrentLimit = new Mutable(log, "stator current limit (a)", statorCurrentLimit, (x) -> currentConfig());
        m_pid = pid;
        // reapply the pid parameters if any change.
        m_pid.register(this::pidConfig);
    }

    /**
     * Makes config synchronous so we can see the errors
     */
    public void longCANTimeout() {
        crash(() -> m_motor.setCANTimeout(500));
    }

    /**
     * Makes everything asynchronous.
     * NOTE: this makes error-checking not work at all.
     */
    public void zeroCANTimeout() {
        crash(() -> m_motor.setCANTimeout(0));
    }

    public void baseConfig() {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.limitSwitch.forwardLimitSwitchEnabled(false);
        conf.limitSwitch.reverseLimitSwitchEnabled(false);
        conf.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
        conf.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
        crash(() -> m_motor.configure(conf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    public void motorConfig() {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.idleMode(switch (m_neutral) {
            case COAST -> IdleMode.kCoast;
            case BRAKE -> IdleMode.kBrake;
        });
        conf.inverted(switch (m_phase) {
            case FORWARD -> false;
            case REVERSE -> true;
        });
        conf.signals.primaryEncoderVelocityPeriodMs(ENCODER_REPORT_PERIOD_MS);
        conf.signals.primaryEncoderVelocityAlwaysOn(true);
        conf.signals.primaryEncoderPositionPeriodMs(ENCODER_REPORT_PERIOD_MS);
        conf.signals.primaryEncoderPositionAlwaysOn(true);
        // slower than default of 10; also affects things like motor temperature and
        // applied output.
        conf.signals.limitsPeriodMs(20);
        crash(() -> m_motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    public void currentConfig() {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.smartCurrentLimit((int) m_statorCurrentLimit.getAsDouble());
        crash(() -> m_motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    /** TODO: don't allow a limit above the configuration */
    public void overrideStatorLimit(int limit) {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.smartCurrentLimit(limit);
        crash(() -> m_motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    /** TODO: make sure clients call this when their need is done */
    public void endCurrentLimitOverride() {
        currentConfig();
    }

    /**
     * The PID parameters here use units of duty cycle per RPM, so for a typical
     * motor speed of a few thousand RPM, and a typical error of a few hundred RPM,
     * a desired control output would be a duty cycle of 0.1 or so, which implies a
     * value of P like 3e-4.
     */
    public void pidConfig() {
        SparkMaxConfig conf = new SparkMaxConfig();
        conf.closedLoop.positionWrappingEnabled(false); // don't use position control
        conf.closedLoop.p(m_pid.getPositionP(), ClosedLoopSlot.kSlot0);
        conf.closedLoop.i(m_pid.getPositionI(), ClosedLoopSlot.kSlot0);
        conf.closedLoop.d(m_pid.getPositionD(), ClosedLoopSlot.kSlot0);
        conf.closedLoop.p(m_pid.getVelocityP(), ClosedLoopSlot.kSlot1);
        conf.closedLoop.i(m_pid.getVelocityI(), ClosedLoopSlot.kSlot1);
        conf.closedLoop.d(m_pid.getVelocityD(), ClosedLoopSlot.kSlot1);
        conf.closedLoop.iZone(m_pid.getPositionIZone(), ClosedLoopSlot.kSlot0);
        conf.closedLoop.iZone(m_pid.getVelocityIZone(), ClosedLoopSlot.kSlot1);
        conf.closedLoop.velocityFF(0, ClosedLoopSlot.kSlot0); // use arbitrary FF instead
        conf.closedLoop.velocityFF(0, ClosedLoopSlot.kSlot1); // use arbitrary FF instead
        conf.closedLoop.outputRange(-1, 1, ClosedLoopSlot.kSlot0);
        conf.closedLoop.outputRange(-1, 1, ClosedLoopSlot.kSlot1);
        crash(() -> m_motor.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    }

    private static void crash(Supplier<REVLibError> s) {
        REVLibError errorCode = s.get();
        if (errorCode != REVLibError.kOk) {
            throw new IllegalStateException(errorCode.name());
        }
    }
}
