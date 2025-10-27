package org.team100.lib.motor.ctre;

import java.util.function.Supplier;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.DoubleCache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.util.CanId;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * Superclass for TalonFX motors.
 * 
 * Relies on Memo and Takt, so you must put Memo.resetAll() and Takt.update() in
 * Robot.robotPeriodic().
 */
public abstract class Talon6Motor implements BareMotor {

    private final TalonFX m_motor;
    private final PhoenixConfigurator m_configurator;
    private final Feedforward100 m_ff;

    // CACHES
    // Two levels of caching here: the cotemporal cache caches the value
    // and also the supplier
    /** position is latency-compensated. */
    protected final DoubleCache m_position;
    protected final DoubleCache m_velocity;
    // protected final DoubleCache m_acceleration;
    protected final DoubleCache m_dutyCycle;
    protected final DoubleCache m_error;
    protected final DoubleCache m_supply;
    protected final DoubleCache m_supplyVoltage;
    protected final DoubleCache m_stator;
    protected final DoubleCache m_temp;
    // protected final DoubleCache m_torque;

    /////////////////////////////////////
    // CONTROL REQUESTS
    //
    // caching the control requests saves allocation
    //
    private final VelocityVoltage m_velocityVoltage;
    private final DutyCycleOut m_dutyCycleOut;
    private final PositionVoltage m_positionVoltage;
    private final MusicTone m_music;

    // LOGGERS
    private final DoubleLogger m_log_desired_duty;
    private final DoubleLogger m_log_desired_position;
    private final DoubleLogger m_log_desired_speed;
    private final DoubleLogger m_log_desired_accel;
    private final DoubleLogger m_log_friction_FF;
    private final DoubleLogger m_log_velocity_FF;
    private final DoubleLogger m_log_accel_FF;
    private final DoubleLogger m_totalFeedForward;
    private final DoubleLogger m_log_torque_FF;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;
    // private final DoubleLogger m_log_accel;
    private final DoubleLogger m_log_output;
    private final DoubleLogger m_log_error;
    private final DoubleLogger m_log_supply;
    private final DoubleLogger m_log_supplyVoltage;
    private final DoubleLogger m_log_stator;
    // private final DoubleLogger m_log_torque;
    private final DoubleLogger m_log_temp;

    protected Talon6Motor(
            LoggerFactory parent,
            CanId canId,
            NeutralMode neutral,
            MotorPhase motorPhase,
            double supplyLimit,
            double statorLimit,
            PIDConstants lowLevelPIDConstants,
            Feedforward100 ff) {
        //////////////////////////////////////
        //
        // CONTROL REQUESTS
        //
        m_velocityVoltage = new VelocityVoltage(0);
        m_dutyCycleOut = new DutyCycleOut(0);
        m_positionVoltage = new PositionVoltage(0);
        m_music = new MusicTone(0);

        //////////////////////////////////////
        // Update frequencies.
        // make control synchronous, i.e. "actuate immediately." See
        // https://github.com/Team254/FRC-2024-Public/blob/040f653744c9b18182be5f6bc51a7e505e346e59/src/main/java/com/team254/lib/ctre/swerve/SwerveModule.java#L210
        m_velocityVoltage.UpdateFreqHz = 0;
        m_dutyCycleOut.UpdateFreqHz = 0;
        m_positionVoltage.UpdateFreqHz = 0;

        LoggerFactory child = parent.type(this);
        m_motor = new TalonFX(canId.id);
        m_ff = ff;

        m_configurator = new PhoenixConfigurator(
                m_motor,
                neutral,
                motorPhase,
                supplyLimit,
                statorLimit,
                lowLevelPIDConstants);
        m_configurator.logCrashStatus();
        m_configurator.baseConfig();
        m_configurator.motorConfig();
        m_configurator.currentConfig();
        m_configurator.pidConfig();
        m_configurator.audioConfig();

        // Cache the status signal getters.
        final StatusSignal<Angle> motorPosition = m_motor.getPosition();
        final StatusSignal<AngularVelocity> motorVelocity = m_motor.getVelocity();
        // final StatusSignal<AngularAcceleration> motorAcceleration =
        // m_motor.getAcceleration();
        final StatusSignal<Double> motorDutyCycle = m_motor.getDutyCycle();
        final StatusSignal<Double> motorClosedLoopError = m_motor.getClosedLoopError();
        final StatusSignal<Current> motorSupplyCurrent = m_motor.getSupplyCurrent();
        final StatusSignal<Voltage> motorSupplyVoltage = m_motor.getSupplyVoltage();
        final StatusSignal<Current> motorStatorCurrent = m_motor.getStatorCurrent();
        final StatusSignal<Temperature> motorDeviceTemp = m_motor.getDeviceTemp();
        // final StatusSignal<Current> motorTorqueCurrent = m_motor.getTorqueCurrent();

        // The memoizer refreshes all the signals at once.
        Cache.registerSignal(motorPosition);
        Cache.registerSignal(motorVelocity);
        // Memo.registerSignal(motorAcceleration);
        Cache.registerSignal(motorDutyCycle);
        Cache.registerSignal(motorClosedLoopError);
        Cache.registerSignal(motorSupplyCurrent);
        Cache.registerSignal(motorSupplyVoltage);
        Cache.registerSignal(motorStatorCurrent);
        Cache.registerSignal(motorDeviceTemp);
        // Memo.registerSignal(motorTorqueCurrent);

        // None of these need to refresh.
        // this latency compensation uses takt time rather than the real clock.
        m_position = Cache
                .ofDouble(() -> {
                    double latency = Utils.fpgaToCurrentTime(Takt.get()) - motorPosition.getTimestamp().getTime();
                    if (latency > 0.04) {
                        System.out.println("WARNING: !!!!!!! stale position! !!!!!!!" + canId);
                        latency = 0.1;
                    }
                    return motorPosition.getValueAsDouble() + (motorVelocity.getValueAsDouble() * latency);
                });
        m_velocity = Cache.ofDouble(() -> motorVelocity.getValueAsDouble());
        m_dutyCycle = Cache.ofDouble(() -> motorDutyCycle.getValueAsDouble());
        // m_acceleration = Memo.ofDouble(() -> motorAcceleration.getValueAsDouble());
        m_error = Cache.ofDouble(() -> motorClosedLoopError.getValueAsDouble());
        m_supply = Cache.ofDouble(() -> motorSupplyCurrent.getValueAsDouble());
        m_supplyVoltage = Cache.ofDouble(() -> motorSupplyVoltage.getValueAsDouble());
        m_stator = Cache.ofDouble(() -> motorStatorCurrent.getValueAsDouble());
        m_temp = Cache.ofDouble(() -> motorDeviceTemp.getValueAsDouble());
        // m_torque = Memo.ofDouble(() -> motorTorqueCurrent.getValueAsDouble());

        m_log_desired_duty = child.doubleLogger(Level.DEBUG, "desired duty cycle [-1,1]");
        m_log_desired_position = child.doubleLogger(Level.DEBUG, "desired position (rev)");
        m_log_desired_speed = child.doubleLogger(Level.DEBUG, "desired speed (rev_s)");
        m_log_desired_accel = child.doubleLogger(Level.TRACE, "desired accel (rev_s2)");
        m_log_friction_FF = child.doubleLogger(Level.TRACE, "friction feedforward (v)");
        m_log_velocity_FF = child.doubleLogger(Level.TRACE, "velocity feedforward (v)");
        m_log_accel_FF = child.doubleLogger(Level.TRACE, "accel feedforward (v)");
        m_log_torque_FF = child.doubleLogger(Level.TRACE, "torque feedforward (v)");
        m_totalFeedForward = child.doubleLogger(Level.TRACE, "total feedforward (v)");

        m_log_position = child.doubleLogger(Level.DEBUG, "position (rev)");
        m_log_velocity = child.doubleLogger(Level.COMP, "velocity (rev_s)");
        // m_log_accel = child.doubleLogger(Level.TRACE, "accel (rev_s2)");
        m_log_output = child.doubleLogger(Level.COMP, "output [-1,1]");
        m_log_error = child.doubleLogger(Level.TRACE, "error (rev_s)");
        m_log_supply = child.doubleLogger(Level.DEBUG, "supply current (A)");
        m_log_supplyVoltage = child.doubleLogger(Level.DEBUG, "supply voltage (V)");
        m_log_stator = child.doubleLogger(Level.DEBUG, "stator current (A)");
        // m_log_torque = child.doubleLogger(Level.TRACE, "torque (Nm)");
        m_log_temp = child.doubleLogger(Level.DEBUG, "temperature (C)");

        child.intLogger(Level.TRACE, "Device ID").log(() -> canId.id);
    }

    /** Set duty cycle immediately. */
    @Override
    public void setDutyCycle(double output) {
        warn(() -> m_motor.setControl(m_dutyCycleOut
                .withOutput(output)));
        m_log_desired_duty.log(() -> output);
        log();
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        int currentA = (int) (torqueNm / kTNm_amp());
        m_configurator.overrideStatorLimit(currentA);
    }

    @Override
    public double getCurrent() {
        return m_motor.getStatorCurrent().getValueAsDouble();
    }

    /**
     * Use VelocityVoltage outboard PID control to hold the given velocity, with
     * friction, velocity, acceleration, and torque feedforwards.
     * 
     * Actuates immediately.
     */
    @Override
    public void setVelocity(double motorRad_S, double motorAccelRad_S2, double motorTorqueNm) {
        final double motorRev_S = motorRad_S / (2 * Math.PI);
        final double motorRev_S2 = motorAccelRad_S2 / (2 * Math.PI);

        final double frictionFFVolts = m_ff.frictionFFVolts(motorRev_S);
        final double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        final double accelFFVolts = m_ff.accelFFVolts(motorRev_S, motorRev_S2);
        final double torqueFFVolts = getTorqueFFVolts(motorTorqueNm);

        final double FFVolts = frictionFFVolts + velocityFFVolts + accelFFVolts + torqueFFVolts;
        // final double FFVolts = torqueFFVolts;

        // VelocityVoltage has an acceleration field for kA feedforward but we use
        // arbitrary feedforward for that.
        warn(() -> m_motor.setControl(
                m_velocityVoltage
                        .withSlot(1)
                        .withVelocity(motorRev_S)
                        .withFeedForward(FFVolts)));

        // without feedforward
        // Phoenix100.warn(() -> m_motor.setControl(
        // m_velocityVoltage
        // .withVelocity(motorRev_S)));

        m_log_desired_speed.log(() -> motorRev_S);
        m_log_desired_accel.log(() -> motorRev_S2);
        m_log_friction_FF.log(() -> frictionFFVolts);
        m_log_velocity_FF.log(() -> velocityFFVolts);
        m_log_accel_FF.log(() -> accelFFVolts);
        m_log_torque_FF.log(() -> torqueFFVolts);
        m_totalFeedForward.log(() -> FFVolts);

        log();
    }

    @Override
    public void play(double freq) {
        m_motor.setControl(m_music.withAudioFrequency(freq));
    }

    /**
     * Use PositionVoltage outboard PID control to hold the given position, with
     * friction, velocity, accel, and torque feedforwards.
     * 
     * Actuates immediately.
     * 
     * Motor revolutions wind up, so setting 0 revs and 1 rev are different.
     */
    @Override
    public void setUnwrappedPosition(
            double positionRad,
            double velocityRad_S,
            double accelRad_S2,
            double torqueNm) {
        final double motorRev = positionRad / (2 * Math.PI);
        final double motorRev_S = velocityRad_S / (2 * Math.PI);
        final double motorRev_S2 = accelRad_S2 / (2 * Math.PI);

        final double frictionFFVolts = m_ff.frictionFFVolts(motorRev_S);
        final double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        final double accelFFVolts = m_ff.accelFFVolts(motorRev_S, motorRev_S2);
        final double torqueFFVolts = getTorqueFFVolts(torqueNm);

        final double FFVolts = frictionFFVolts + velocityFFVolts + accelFFVolts + torqueFFVolts;

        // PositionVoltage has a velocity field for kV feedforward but we use arbitrary
        // feedforward for that.
        warn(() -> m_motor.setControl(
                m_positionVoltage
                        .withSlot(0)
                        .withPosition(motorRev)
                        .withFeedForward(FFVolts)));

        m_log_desired_position.log(() -> motorRev);
        m_log_desired_speed.log(() -> motorRev_S);
        m_log_desired_accel.log(() -> motorRev_S2);
        m_log_friction_FF.log(() -> frictionFFVolts);
        m_log_velocity_FF.log(() -> velocityFFVolts);
        m_log_torque_FF.log(() -> torqueFFVolts);
        m_log_accel_FF.log(() -> accelFFVolts);
        m_totalFeedForward.log(() -> FFVolts);

        log();
    }

    /** Value is updated in Robot.robotPeriodic(). */
    @Override
    public double getVelocityRad_S() {
        return getVelocityRev_S() * 2 * Math.PI;
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void reset() {
        m_position.reset();
        m_velocity.reset();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    /**
     * Sets integrated sensor position to zero.
     * 
     * Note this takes **FOREVER**, like tens of milliseconds, so you can only do it
     * at startup.
     */
    public void resetEncoderPosition() {
        System.out.println("WARNING: Setting CTRE encoder position is very slow!");
        warn(() -> m_motor.setPosition(0, 1));
        m_position.reset();
        m_velocity.reset();
    }

    /**
     * Set integrated sensor position in rotations.
     * 
     * This is the "unwrapped" position, i.e. the domain is infinite, not cyclical
     * within +/- pi.
     * 
     * Note this takes **FOREVER**, like tens of milliseconds, so you can only do it
     * at startup.
     */
    private void setUnwrappedEncoderPosition(double motorPositionRev) {
        System.out.println("WARNING: Setting CTRE encoder position is very slow!");
        warn(() -> m_motor.setPosition(motorPositionRev, 1));
    }

    /**
     * Set integrated sensor position in radians.
     * 
     * This is the "unwrapped" position, i.e. the domain is infinite, not cyclical
     * within +/- pi.
     * 
     * Note this takes **FOREVER**, like tens of milliseconds, so you can only do it
     * at startup.
     */
    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        double motorPositionRev = positionRad / (2.0 * Math.PI);
        setUnwrappedEncoderPosition(motorPositionRev);
    }

    /**
     * Not latency-compensated.
     * Updated in Robot.robotPeriodic().
     */
    public double getVelocityRev_S() {
        return m_velocity.getAsDouble();
    }

    /**
     * Returns the "unwrapped" position, i.e. the domain is infinite, not cyclical
     * within +/- pi.
     * 
     * Latency-compensated, represents the current Takt.
     * Updated in `Robot.robotPeriodic()`.
     */
    public double getUnwrappedPositionRev() {
        return m_position.getAsDouble();
    }

    /**
     * This is the "unwrapped" position, i.e. the domain is infinite, not cyclical
     * within +/- pi.
     */
    @Override
    public double getUnwrappedPositionRad() {
        double motorPositionRev = getUnwrappedPositionRev();
        double positionRad = motorPositionRev * 2 * Math.PI;
        return positionRad;
    }

    /** ait a long time for a new value, do not use outside testing. */
    public double getUnwrappedPositionBlockingRev() {
        return m_motor.getPosition().waitForUpdate(1).getValueAsDouble();
    }

    protected void log() {
        m_log_position.log(m_position);
        m_log_velocity.log(m_velocity);
        // m_log_accel.log(m_acceleration);
        m_log_output.log(m_dutyCycle);
        m_log_error.log(m_error);
        m_log_supply.log(m_supply);
        m_log_supplyVoltage.log(m_supplyVoltage);
        m_log_stator.log(m_stator);
        // m_log_torque.log(this::getMotorTorque);
        m_log_temp.log(m_temp);
    }

    // private double getMotorTorque() {
    // // I looked into latency compensation of this signal but it doesn't seem
    // // possible. latency compensation requires a signal and its time derivative,
    // // e.g. position and velocity, or yaw and angular velocity. There doesn't
    // seem
    // // to be such a thing for current.
    // return m_torque.getAsDouble() * kTNm_amp();
    // }

    @Override
    public void periodic() {
        log();
    }

    /////////////////////////////////////////////

    private static void warn(Supplier<StatusCode> s) {
        StatusCode statusCode = s.get();
        if (statusCode.isError()) {
            System.out.println("WARNING: " + statusCode.toString());
        }
    }
}
