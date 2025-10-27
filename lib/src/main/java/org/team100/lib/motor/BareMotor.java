package org.team100.lib.motor;

import org.team100.lib.music.Player;

/**
 * Methods pertain only to the output shaft, not the motion of the attached
 * mechanism. Accordingly, the units are always rotational, and there should be
 * no gear ratios in any implementation.
 */
public interface BareMotor extends Player {

    /**
     * Some motors allow torque limiting through current limiting.
     * 
     * NOTE! Changing current limits can be a slow operation, so don't do this too
     * often.
     */
    void setTorqueLimit(double torqueNm);

    /**
     * Open-loop duty cycle control.
     * 
     * @param output in range [-1, 1]
     */
    void setDutyCycle(double output);

    /**
     * Velocity feedback with friction, velocity, acceleration, and holding torque.
     * 
     * Could be open-loop (e.g. "kV") or closed-loop.
     * 
     * @param velocityRad_S motor shaft speed, rad/s.
     * @param accelRad_S2   rad/s^2.
     * @param torqueNm      Nm, for gravity compensation or holding.
     */
    void setVelocity(
            double velocityRad_S,
            double accelRad_S2,
            double torqueNm);

    /**
     * Value should be updated in Robot.robotPeriodic().
     * 
     * Motor shaft speed.
     */
    double getVelocityRad_S();

    /**
     * Returns the "unwrapped" angular position, i.e. the measurement domain
     * continues beyond +/- pi.
     * 
     * Value should be updated in Robot.robotPeriodic().
     * 
     * Motor shaft position.
     */
    double getUnwrappedPositionRad();

    /** Motor stator current in amps. */
    double getCurrent();

    /**
     * This is the "unwrapped" position, i.e. the domain is infinite, not cyclical
     * within +/- pi.
     */
    void setUnwrappedEncoderPositionRad(double positionRad);

    /**
     * Position feedback with feedforward for friction, velocity, acceleration, and
     * holding torque.
     * 
     * Revolutions wind up; 0 != 2pi.
     * 
     * This is the "unwrapped" position, i.e. the domain is infinite, not cyclical
     * within +/- pi
     * 
     * Should actuate immediately.
     * 
     * Make sure you don't double-count factors of torque/accel.
     * 
     * 
     * @param positionRad   radians.
     * @param velocityRad_S rad/s.
     * @param accelRad_S2   rad/s^2.
     * @param torqueNm      Nm, for gravity compensation or holding.
     */
    void setUnwrappedPosition(
            double positionRad,
            double velocityRad_S,
            double accelRad_S2,
            double torqueNm);

    /**
     * Motor resistance in ohms, used to calculate voltage from desired torque
     * current.
     */
    double kROhms();

    /**
     * Motor torque constant, kT, in Nm per amp, used to calculate current from
     * desired torque.
     */
    double kTNm_amp();

    /**
     * Incremental voltage required to produce the given torque, used for
     * feedforward.
     */
    default double getTorqueFFVolts(double torqueNm) {
        double torqueFFAmps = torqueNm / kTNm_amp();
        return torqueFFAmps * kROhms();
    }

    /** This is not "hold position" this is "torque off". */
    void stop();

    /** Reset the cache. */
    void reset();

    /**
     * For test cleanup.
     */
    void close();

    /** For logging */
    void periodic();

}
