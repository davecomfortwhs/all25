package org.team100.lib.motor.ctre;

import java.util.function.Supplier;

import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Utilities for CTRE Phoenix motors: Falcon, Kraken. */
public class PhoenixConfigurator {
    private static final boolean ACTUALLY_CRASH = false;
    /**
     * The default is 0.05. This is much longer, to eliminate unnecessary config
     * failures.
     */
    private static final double TIMEOUT_SEC = 0.3;
    // speeding up the updates is a tradeoff between latency and CAN utilization.
    // 254 seems to think that 100 is a good compromise?
    // see
    // https://github.com/Team254/FRC-2024-Public/blob/040f653744c9b18182be5f6bc51a7e505e346e59/src/main/java/com/team254/lib/ctre/swerve/SwerveDrivetrain.java#L382
    private static final int SIGNAL_UPDATE_FREQ_HZ = 100;

    private final TalonFX m_motor;
    private final NeutralMode m_neutral;
    private final MotorPhase m_phase;
    private final double m_supply;
    private final double m_stator;
    private final PIDConstants m_pid;

    public PhoenixConfigurator(
            TalonFX motor,
            NeutralMode neutral,
            MotorPhase phase,
            double supply,
            double stator,
            PIDConstants pid) {
        m_motor = motor;
        m_neutral = neutral;
        m_phase = phase;
        m_supply = supply;
        m_stator = stator;
        m_pid = pid;
        // reapply the pid parameters if any change.
        m_pid.register(this::pidConfig);
    }

    public void logCrashStatus() {
        if (ACTUALLY_CRASH)
            System.out.println("WARNING: ***** Config fail will CRASH the robot, NOT FOR COMP!");
        else
            System.out.println("***** Config fail will not be caught, NOT FOR DEV!");

    }

    public static void crash(Supplier<StatusCode> s) {
        StatusCode statusCode = s.get();
        if (statusCode.isError()) {
            if (ACTUALLY_CRASH)
                throw new IllegalStateException(statusCode.toString());
            System.out.println("WARNING: ******************************************************");
            System.out.println("WARNING: ****** MOTOR CONFIG HAS FAILED MOTOR IS NOT SET CORRECTLY ******");
            System.out.println("WARNING: " + statusCode.toString());
        }
    }

    public void baseConfig() {
        TalonFXConfiguration base = new TalonFXConfiguration();
        crash(() -> m_motor.getConfigurator().apply(base, TIMEOUT_SEC));
    }

    public void motorConfig() {
        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.NeutralMode = switch (m_neutral) {
            case COAST -> NeutralModeValue.Coast;
            case BRAKE -> NeutralModeValue.Brake;
        };
        motorConfigs.Inverted = switch (m_phase) {
            case FORWARD -> InvertedValue.CounterClockwise_Positive;
            case REVERSE -> InvertedValue.Clockwise_Positive;
        };
        crash(() -> m_motor.getConfigurator().apply(motorConfigs, TIMEOUT_SEC));
        crash(() -> m_motor.getPosition().setUpdateFrequency(SIGNAL_UPDATE_FREQ_HZ));
        crash(() -> m_motor.getVelocity().setUpdateFrequency(SIGNAL_UPDATE_FREQ_HZ));
        crash(() -> m_motor.getAcceleration().setUpdateFrequency(SIGNAL_UPDATE_FREQ_HZ));
        crash(() -> m_motor.getTorqueCurrent().setUpdateFrequency(SIGNAL_UPDATE_FREQ_HZ));
    }

    /**
     * @see https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
     * @see https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
     */
    public void currentConfig() {
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.SupplyCurrentLimit = m_supply;
        currentConfigs.SupplyCurrentLimitEnable = true;
        currentConfigs.StatorCurrentLimit = m_stator;
        currentConfigs.StatorCurrentLimitEnable = true;
        crash(() -> m_motor.getConfigurator().apply(currentConfigs, TIMEOUT_SEC));
    }

    /** TODO: don't allow a limit above the configuration */
    public void overrideStatorLimit(double limit) {
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.SupplyCurrentLimit = m_supply;
        currentConfigs.SupplyCurrentLimitEnable = true;
        currentConfigs.StatorCurrentLimit = limit;
        currentConfigs.StatorCurrentLimitEnable = true;
        crash(() -> m_motor.getConfigurator().apply(currentConfigs, TIMEOUT_SEC));
    }

    /** TODO: make sure clients call this when their need is done */
    public void endCurrentLimitOverride() {
        currentConfig();
    }

    /**
     * Parameter units depend on the mode. We use velocityvoltage for velocity
     * control, so the units would be volts per rev/s. For position control we use
     * positionvoltage, so the units would be volts per revolution.
     */
    public void pidConfig() {
        Slot0Configs slot0Configs = new Slot0Configs();
        Slot1Configs slot1Configs = new Slot1Configs();
        slot0Configs.kV = 0.0; // we use "arbitrary feedforward", not this.
        slot1Configs.kV = 0.0;
        slot0Configs.kP = m_pid.getPositionP();
        slot0Configs.kI = m_pid.getPositionI();
        slot0Configs.kD = m_pid.getPositionD();
        slot1Configs.kP = m_pid.getVelocityP();
        slot1Configs.kI = m_pid.getVelocityI();
        slot1Configs.kD = m_pid.getVelocityD();
        crash(() -> m_motor.getConfigurator().apply(slot0Configs, TIMEOUT_SEC));
        crash(() -> m_motor.getConfigurator().apply(slot1Configs, TIMEOUT_SEC));
    }

    /** Allow music during disable: this is for warning sounds. */
    public void audioConfig() {
        AudioConfigs audioConfigs = new AudioConfigs();
        audioConfigs.AllowMusicDurDisable = true;
        crash(() -> m_motor.getConfigurator().apply(audioConfigs, TIMEOUT_SEC));
    }
}
