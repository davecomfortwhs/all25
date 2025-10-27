package org.team100.lib.controller.r3;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;

/**
 * Known-good controller settings.
 */
public class ControllerFactoryR3 {

    /** For real robots. */
    public static ControllerR3 byIdentity(LoggerFactory log) {
        switch (Identity.instance) {
            case COMP_BOT -> {
                return new FullStateControllerR3(log, 2.9, 3.5, 0.025, 0.01, 0.02, 0.3, 1, 1);
                // return new FullStateSwerveController(log, 0, 0, 0, 0, 0, 0, 0, 0);
            }
            case SWERVE_ONE -> {
                return new FullStateControllerR3(log, 3, 3.5, 0.05, 0, 0.01, 0.01, 1, 1);
            }
            case SWERVE_TWO -> {
                return new FullStateControllerR3(log, 4, 4, 0.25, 0.25, 0.01, 0.02, 0.01, 0.02);
            }
            default -> {
                return new FullStateControllerR3(log, 3.0, 3.5, 0, 0, 0.01, 0.01, 0.01, 0.01);
            }
        }
    }

    public static FullStateControllerR3 ridiculous(LoggerFactory log) {
        return new FullStateControllerR3(log, 3, 3, 0.1, 0.1, 0.01, 0.01, 0.01, 0.01);
    }

    public static FullStateControllerR3 fieldRelativeFancyPIDF(LoggerFactory log) {
        return new FullStateControllerR3(log, 2.4, 1.3, 0.1, 0.1, 0.01, 0.02, 0.01, 0.02);
    }

    public static FullStateControllerR3 fieldRelativeGoodPIDF(LoggerFactory log) {
        return new FullStateControllerR3(log, 1, 1.3, 0.1, 0.1, 0.01, 0.02, 0.01, 0.02);
    }

    public static FullStateControllerR3 autoFieldRelativePIDF(LoggerFactory log) {
        return new FullStateControllerR3(log, 1.5, 1.3, 0, 0, 0.1, 0.1, 0.1, 0.1);
    }

    public static FullStateControllerR3 auto2025LooseTolerance(LoggerFactory log) {
        return new FullStateControllerR3(log,
                7.2, // p cartesian
                3.5, // p theta
                0.055, // p cartesian v
                0.01, // ptheta v
                0.035, // x tol
                0.1, // theta tol
                1, // xdot tol
                1); // omega tol
    }

    /** pick tolerance is *very* loose */
    public static FullStateControllerR3 pick(LoggerFactory log) {
        return new FullStateControllerR3(log,
                7.2, // p cartesian
                3.5, // p theta
                0.055, // p cartesian v
                0.01, // ptheta v
                0.15, // x tol
                0.4, // theta tol
                4, // xdot tol
                4); // omega tol
    }

    ////////////////
    //
    // don't use these for real robots
    //

    public static FullStateControllerR3 testFieldRelativePIDF(LoggerFactory log) {
        return new FullStateControllerR3(log, 2.4, 2.4, 0.1, 0.1, 0.01, 0.02, 0.01, 0.02);
    }

    public static FullStateControllerR3 testFieldRelativeFFOnly(LoggerFactory log) {
        return new FullStateControllerR3(log, 0, 0, 0, 0, 0.01, 0.02, 0.01, 0.02);
    }

    public static FullStateControllerR3 test(LoggerFactory log) {
        return new FullStateControllerR3(log, 3.0, 3.5, 0, 0, 0.01, 0.01, 0.01, 0.01);
    }

    /** high gains used in tests. */
    public static FullStateControllerR3 test2(LoggerFactory log) {
        return new FullStateControllerR3(log, 4, 4, 0.25, 0.25, 0.01, 0.02, 0.01, 0.02);
    }

    private ControllerFactoryR3() {
        // don't call this
    }

}
