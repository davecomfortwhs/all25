package org.team100.lib.motion.servo;

import org.team100.lib.music.Player;

/**
 * Represents a servo whose output is measured in linear units -- this is
 * usually relevant for wheeled mechanisms, where the surface speed of the wheel
 * is the important thing. Examples:
 * 
 * A conveyor belt drive is a wheel, but the important thing is the linear
 * movement of the belt.
 * 
 * A ball-shooter is a wheel, but the important thing is its surface speed.
 */
public interface LinearVelocityServo extends Player {
    void reset();

    void setDutyCycle(double dutyCycle);

    /**
     * There's no profile here, it just sets the mechanism velocity.
     * 
     * Set velocity and compute implied acceleration based on the previous call,
     * using TimedRobot100.LOOP_PERIOD_S. If you call this more often, you'll
     * get weird results.
     * 
     * @param setpointM_S desired speed, m/s
     */
    void setVelocity(double setpointM_S);

    /**
     * @param setpointM_S  desired speed, m/s
     * @param setpointM_S2 desired acceleration m/s^2
     */
    void setVelocity(double setpointM_S, double setpointM_S2);

    /** meters/sec. Note this can be noisy, maybe filter it. */
    double getVelocity();

    /** we're within some tolerance of the desired */
    boolean atGoal();

    void stop();

    /** meters. Implementations should use the Cache mechanism. */
    double getDistance();

    /** For logging */
    void periodic();

}
