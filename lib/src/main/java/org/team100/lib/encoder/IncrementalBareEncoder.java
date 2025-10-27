package org.team100.lib.encoder;

/** Represents motor-shaft encoder, probably some kind of built-in. */
public interface IncrementalBareEncoder {

    /**
     * Returns the "unwrapped" angular position, i.e. the measurement domain
     * continues beyond +/- pi.
     * 
     * Value should be updated in Robot.robotPeriodic().
     * 
     * @return rad
     */
    double getUnwrappedPositionRad();

    /**
     * Value should be updated in Robot.robotPeriodic().
     * 
     * Note some rate implementations can be noisy.
     * 
     * @return rad/s
     */
    double getVelocityRad_S();

    /**
     * Resets position to zero.
     *
     * Caches should also be flushed, so the new value is available immediately.
     */
    void reset();

    /**
     * Releases the encoder resource, if necessary (e.g. HAL ports).
     */
    void close();

    /**
     * Sets the incremental encoder position. This is only used to "zero" it, and
     * only done by the ProxyRotaryPositionSensor.
     * 
     * This is the "unwrapped" position, i.e. the domain is infinite, not cyclical
     * within +/- pi.
     * 
     * This is very slow, only use it on startup.
     * 
     * Caches should also be flushed, so the new value is available immediately.
     */
    void setUnwrappedEncoderPositionRad(double motorPositionRad);

    /** For logging */
    void periodic();

}
