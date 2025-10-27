package org.team100.lib.experiments;

/**
 * An experiment is something that can be selectively enabled.
 * 
 * TODO: delete stale experiments.
 */
public enum Experiment {
    /**
     * Smooth chassis speeds.
     */
    UseSetpointGenerator,
    /**
     * Flush network tables as often as possible. Do not enable this experiment in
     * competition, you'll overwhelm the network and the RIO
     */
    FlushOften,
    /**
     * Pay attention to camera input. It's useful to turn this off for testing and
     * calibration.
     */
    HeedVision,
    /**
     * Control heading when manually steering in snaps mode, to prevent
     * rotational drifting.
     */
    StickyHeading,
    /**
     * Use softer vision update gains
     */
    AvoidVisionJitter,
    /**
     * Filter snap rotational output to remove oscillation
     */
    SnapThetaFilter,
    /**
     * Make the kinodynamic limiter prefer rotation.
     */
    LimitsPreferRotation,
    /**
     * Clip the snap omega
     */
    SnapGentle,
    /**
     * Ignore very small inputs, to reduce jittering.
     */
    SwerveDeadband,
    /**
     * Help drive motors overcome steering.
     * TODO(: make this the default?
     */
    CorrectSpeedForSteering,
    /**
     * Use the steering error to adjust the drive motor speed, minimizing
     * cross-track error on a per-module basis.
     * 
     * This could be the "cosine" used by some teams; our implementation is a narrow
     * gaussian. The idea is simple: avoid driving in the wrong direction. In this
     * case, since the upper layers are unaware of it, it will introduce lag (thus
     * controller error) into timed movements, and also, because the drive
     * corrections are uncoordinated, this produces rotational errors, not just
     * translational ones.
     * 
     * But it has the benefit of being simple.
     * TODO: make this the default?
     */
    ReduceCrossTrackError,
    /**
     * Use pure outboard PID for steering control, rather than the usual profiled
     * motion -- it's faster and less work for the RoboRIO.
     * TODO: make this the default?
     */
    UnprofiledSteering,
}
