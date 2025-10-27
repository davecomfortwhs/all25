package org.team100.lib.profile;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.incremental.CurrentLimitedExponentialProfile;
import org.team100.lib.profile.incremental.IncrementalProfile;
import org.team100.lib.profile.incremental.TrapezoidIncrementalProfile;
import org.team100.lib.profile.incremental.TrapezoidProfileWPI;
import org.team100.lib.state.Control100;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.Model100;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.math.MathUtil;

/**
 * Coordinates three axes so that their profiles complete at about the same
 * time, by adjusting the maximum allowed acceleration.
 * 
 * Note that because acceleration is adjusted, but not cruise velocity, the
 * resulting paths will not be straight, for rest-to-rest profiles.
 */
public class HolonomicProfile {
    private enum Profile {
        WPI,
        P100,
        EXP
    }

    private static final Profile PROFILE = Profile.P100;
    /** For testing */
    private static final boolean DEBUG = false;
    /** Solver accuracy is low, in the interest of speed. */
    private static final double ETA_TOLERANCE = 0.1;
    /** Simulation for ETA is coarse, in the interest of speed. */
    private static final double SOLVE_DT = 0.1;
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;

    private final IncrementalProfile px;
    private final IncrementalProfile py;
    private final IncrementalProfile ptheta;

    // package-private for testing
    IncrementalProfile ppx;
    IncrementalProfile ppy;
    IncrementalProfile pptheta;
    // for testing only
    double sx;
    double sy;
    double stheta;

    private HolonomicProfile(
            IncrementalProfile px,
            IncrementalProfile py,
            IncrementalProfile ptheta) {
        this.px = px;
        this.py = py;
        this.ptheta = ptheta;
        ppx = px;
        ppy = py;
        pptheta = ptheta;
    }

    /**
     * Make a holonomic profile using the kinodynamic absolute maxima, scaled as
     * specified.
     */
    public static HolonomicProfile get(
            LoggerFactory logger,
            SwerveKinodynamics kinodynamics,
            double vScale,
            double aScale,
            double omegaScale,
            double alphaScale) {
        logger.name("HolonomicProfile").stringLogger(Level.TRACE, "profile").log(() -> PROFILE.name());
        switch (PROFILE) {
            case EXP -> {
                return currentLimitedExponential(
                        kinodynamics.getMaxDriveVelocityM_S() * vScale,
                        kinodynamics.getMaxDriveAccelerationM_S2() * aScale,
                        kinodynamics.getStallAccelerationM_S2() * aScale,
                        kinodynamics.getMaxAngleSpeedRad_S() * omegaScale,
                        kinodynamics.getMaxAngleAccelRad_S2() * alphaScale,
                        kinodynamics.getMaxAngleStallAccelRad_S2() * alphaScale);
            }

            case WPI -> {
                return wpi(
                        kinodynamics.getMaxDriveVelocityM_S() * vScale,
                        kinodynamics.getMaxDriveAccelerationM_S2() * aScale,
                        kinodynamics.getMaxAngleSpeedRad_S() * omegaScale,
                        kinodynamics.getMaxAngleAccelRad_S2() * alphaScale);
            }

            case P100 -> {
                return trapezoidal(
                        kinodynamics.getMaxDriveVelocityM_S() * vScale,
                        kinodynamics.getMaxDriveAccelerationM_S2() * aScale,
                        0.05,
                        kinodynamics.getMaxAngleSpeedRad_S() * omegaScale,
                        kinodynamics.getMaxAngleAccelRad_S2() * alphaScale,
                        0.1);
            }

            default -> {
                return null;
            }
        }
    }

    public static HolonomicProfile wpi(
            double maxXYVel,
            double maxXYAccel,
            double maxAngularVel,
            double maxAngularAccel) {
        return new HolonomicProfile(
                new TrapezoidProfileWPI(maxXYVel, maxXYAccel),
                new TrapezoidProfileWPI(maxXYVel, maxXYAccel),
                new TrapezoidProfileWPI(maxAngularVel, maxAngularAccel));
    }

    public static HolonomicProfile trapezoidal(
            double maxXYVel,
            double maxXYAccel,
            double xyTolerance,
            double maxAngularVel,
            double maxAngularAccel,
            double angularTolerance) {
        return new HolonomicProfile(
                new TrapezoidIncrementalProfile(maxXYVel, maxXYAccel, xyTolerance),
                new TrapezoidIncrementalProfile(maxXYVel, maxXYAccel, xyTolerance),
                new TrapezoidIncrementalProfile(maxAngularVel, maxAngularAccel, angularTolerance));
    }

    public static HolonomicProfile currentLimitedExponential(
            double maxXYVel,
            double limitedXYAccel,
            double stallXYAccel,
            double maxAngularVel,
            double limitedAlpha,
            double stallAlpha) {
        return new HolonomicProfile(
                new CurrentLimitedExponentialProfile(maxXYVel, limitedXYAccel, stallXYAccel),
                new CurrentLimitedExponentialProfile(maxXYVel, limitedXYAccel, stallXYAccel),
                new CurrentLimitedExponentialProfile(maxXYVel, limitedAlpha, stallAlpha));
    }

    /**
     * Find scale factors that make the axes finish around the same time.
     * 
     * This is a fairly coarse optimization, i.e. ETA within 0.1 sec or so, Reset
     * the scale factors.
     * 
     * @param i initial
     * @param g goal
     */
    public void solve(ModelR3 i, ModelR3 g) {
        // first find the max ETA
        if (DEBUG) {
            System.out.printf("i %s g %s\n", i, g);
        }
        // note coarser DT
        double xETA = px.simulateForETA(SOLVE_DT, i.x().control(), g.x());
        double yETA = py.simulateForETA(SOLVE_DT, i.y().control(), g.y());
        double thetaETA = ptheta.simulateForETA(SOLVE_DT, i.theta().control(), g.theta());

        if (DEBUG) {
            System.out.printf("ETAs: %f %f %f\n", xETA, yETA, thetaETA);
        }
        double slowETA = xETA;
        slowETA = Math.max(slowETA, yETA);
        slowETA = Math.max(slowETA, thetaETA);

        sx = px.solve(SOLVE_DT, i.x().control(), g.x(), slowETA, ETA_TOLERANCE);
        sy = py.solve(SOLVE_DT, i.y().control(), g.y(), slowETA, ETA_TOLERANCE);
        stheta = ptheta.solve(SOLVE_DT, i.theta().control(), g.theta(), slowETA, ETA_TOLERANCE);

        if (DEBUG) {
            System.out.printf("sx %.3f sy %.3f stheta %.3f\n", sx, sy, stheta);
        }

        ppx = px.scale(sx);
        ppy = py.scale(sy);
        pptheta = ptheta.scale(stheta);
    }

    /**
     * Compute the control for the end of the next time step.
     * 
     * @param i initial
     * @param g goal
     * @return control
     */
    public ControlR3 calculate(ModelR3 i, ModelR3 g) {
        if (i == null || g == null) {
            // this can happen on startup when the initial state hasn't yet been defined,
            // but the cache refresher is trying to update the references.
            return ControlR3.zero();
        }
        if (DEBUG) {
            System.out.printf("initial %s goal %s\n", i, g);
        }
        Control100 stateX = ppx.calculate(DT, i.x().control(), g.x());
        Control100 stateY = ppy.calculate(DT, i.y().control(), g.y());
        // theta is periodic; choose a setpoint angle near the goal.
        Model100 theta = new Model100(
                MathUtil.angleModulus(i.theta().x() - g.theta().x()) + g.theta().x(),
                i.theta().v());
        Control100 stateTheta = pptheta.calculate(DT, theta.control(), g.theta());
        return new ControlR3(stateX, stateY, stateTheta);
    }
}
