package org.team100.lib.motion.swerve.kinodynamics;

import org.team100.lib.config.Identity;

/**
 * Each drivetrain should be tuned, and the values here should be the physical
 * maxima.
 * 
 * FYI according to their 2022 code, 254's max speed in 2022 was 5.05 m/s, which
 * is about the same as ours, but their max acceleration was 4.4 m/s^2, which is
 * crazy quick.
 *
 * Tune these limits to match the absolute maximum possible performance of the
 * drivetrain, not what seems "comfortable."
 * 
 * Do not use this class to configure driver preferences, use a command or
 * control instead.
 * 
 * In particular, the maximum spin rate is likely to seem quite high. Do not
 * lower it here.
 */
public class SwerveKinodynamicsFactory {
    /**
     * Supply the vertical center of mass based on the elevator position.
     * 
     * Tests should try to avoid calling get(). Use one of the test-specific methods
     * below instead.
     */
    public static SwerveKinodynamics get() {
        System.out.printf("Swerve Kinodynamics Factory using Identity %s\n", Identity.instance);
        switch (Identity.instance) {
            case COMP_BOT:
                // these numbers are a guess based on the betabot numbers.
                // the comp bot uses the "fast" ratio and FOC falcons
                // so should be a bit higher top speed and less acceleration.
                // note these measurements were updated jun 24.
                // 9/24/04, raised steering rate from 20 to 40, accel from 60 to 120.
                return new SwerveKinodynamics(
                        4.5, // max vel m/s
                        10, // stall m/s/s
                        5, // max accel m/s/s
                        25, // max decel m/s/s
                        40, // max module steering rate rad/s
                        120, // max module steering accel rad/s/s
                        0.590, // front track m
                        0.590, // back track m
                        0.590, // wheelbase m
                        0.295275, // front offset m
                        0.5); // vcg m NOTE VERY HIGH
            case SWERVE_TWO:
                return new SwerveKinodynamics(
                        4, // vel m/s
                        10, // stall m/s/s
                        2, // accel m/s/s
                        2, // decel m/s/s
                        13, // steering rate rad/s
                        20 * Math.PI, // steering accel rad/s/s
                        0.380, // track m
                        0.380, // track m
                        0.445, // wheelbase m
                        0.2225, // front offset m
                        0.5); // vcg m HIGH LIKE COMP
            case SWERVE_ONE:
                return new SwerveKinodynamics(
                        5, // vel m/s
                        10, // stall m/s/s
                        10, // max accel m/s/s
                        20, // max decel m/s/s
                        40, // steering rate rad/s
                        40 * Math.PI, // steering accel rad/s/s
                        0.49, // front track m
                        0.44, // back track m
                        0.462, // wheelbase m
                        0.31, // front offset m
                        0.5); // vcg m HIGH LIKE COMP
            case BLANK:
                // this is used for tests and simulation; the limits should be kept in sync
                // with the comp config, so that the simulator provides realistic
                // feedback. it's not *identical* to the comp config because it affects
                // a whole lots of tests, which you'll have to touch every time you
                // change it. :-(
                return new SwerveKinodynamics(
                        3, // vel m/s
                        10, // stall m/s/s
                        5, // accel m/s/s
                        25, // decel m/s/s
                        40, // steering rate rad/s
                        120, // steering accel rad/s/s
                        0.5, // track m
                        0.5, // track m
                        0.5, // wheelbase m
                        0.25, // front offset m
                        0.1); // vcg m
            case BETA_BOT:
                // these numbers were extracted from module mode acceleration
                // runs as shown in this spreadsheet
                // https://docs.google.com/spreadsheets/d/1x0WEDIYosVBrsz37VXPEEmLB6-AuLnmwBp_mgozKFI0
                // the actual profile is exponential. these numbers represent the maximum
                // tangent
                // so that the result will be snappy at low speed, and unable to meet its
                // setpoints at high speed.
                // note the betabot uses the "medium" speed ratio
                // and falcons with FOC -- these data were taken when the gear ratio was
                // misconfigured so i reduced them accordingly.
                // also i observed the steering speed and reduced it a bit.
                // the beta bot has very low VCG.
                return new SwerveKinodynamics(
                        5, // max vel m/s
                        10, // stall m/s/s
                        20, // max accel m/s/s
                        50, // max decel m/s/s
                        20, // max module steering rate rad/s
                        60, // max module steering accel rad/s/s
                        0.491, // front track m
                        0.44, // back track m
                        0.491, // wheelbase m
                        0.29, // front offset m
                        0.5); // vcg m HIGH LIKE COMP
            default:
                System.out.println("WARNING: ***");
                System.out.println("WARNING: *** Using default kinodynamics, this should never happen.");
                System.out.println("WARNING: ***");
                return new SwerveKinodynamics(
                        5, // vel m/s
                        20, // stall m/s/s
                        5, // accel m/s/s
                        5, // decel m/s/s
                        13, // steering rate rad/s
                        20 * Math.PI, // steering accel rad/s/s
                        0.5, // track m
                        0.5, // track m
                        0.5, // wheelbase m
                        0.25, // front offset m
                        0.3); // vcg m
        }
    }

    /** This is for the Mecanum drive on Rookiebot 2 */
    public static SwerveKinodynamics mecanum() {
        return new SwerveKinodynamics(
                5, // vel m/s
                10, // stall m/s/s
                10, // accel m/s/s
                20, // decel m/s/s
                20 * Math.PI,
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // track m
                0.5, // wheelbase m
                0.25, // front offset m
                0.3); // vcg m
    }

    /** This is for the tank drive on Rookiebot 1 */
    public static SwerveKinodynamics tank() {
        return new SwerveKinodynamics(
                5, // vel m/s
                10, // stall m/s/s
                10, // accel m/s/s
                20, // decel m/s/s
                20 * Math.PI,
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // track m
                0.5, // wheelbase m
                0.25, // front offset m
                0.3); // vcg m
    }

    /**
     * This contains garbage values, not for anything real.
     * 
     * In particular, the steering rate is *very* slow, which might be useful if
     * you're wanting to allow for steering delay.
     */
    public static SwerveKinodynamics forTest() {
        return new SwerveKinodynamics(
                1, // vel m/s
                10, // stall m/s/s
                1, // accel m/s/s
                1, // decel m/s/s
                20 * Math.PI,
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // track m
                0.5, // wheelbase m
                0.25, // front offset m
                0.3); // vcg m
    }

    public static SwerveKinodynamics forRealisticTest() {
        return new SwerveKinodynamics(
                5, // vel m/s
                10, // stall m/s/s
                10, // accel m/s/s
                20, // decel m/s/s
                20 * Math.PI,
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // track m
                0.5, // wheelbase m
                0.25, // front offset m
                0.3); // vcg m
    }

    public static SwerveKinodynamics forTrajectoryTimingTest() {
        return new SwerveKinodynamics(
                3.5, // vel m/s
                20, // stall m/s/s
                10, // accel m/s/s
                10, // decel m/s/s
                20 * Math.PI,
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // track m
                0.5, // wheelbase m
                0.25, // front offset m
                0.3); // vcg m
    }

    public static SwerveKinodynamics likeComp25() {
        return new SwerveKinodynamics(
                3, // max vel m/s
                10, // stall m/s/s
                5, // max accel m/s/s
                25, // max decel m/s/s
                40, // max module steering rate rad/s
                120, // max module steering accel rad/s/s
                0.590, // front track m
                0.590, // back track m
                0.590, // wheelbase m
                0.295275, // front offset m
                0.5); // m NOTE VERY HIGH
    }

    public static SwerveKinodynamics forTest2() {
        return new SwerveKinodynamics(
                2, // vel m/s
                5, // stall m/s/s
                1, // accel m/s/s
                1, // decel m/s/s
                1, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // track m
                0.5, // wheelbase m
                0.25, // front offset m
                0.6); // vcg m
    }

    public static SwerveKinodynamics forTest3() {
        return new SwerveKinodynamics(
                2, // vel m/s
                5, // stall m/s/s
                2, // accel m/s/s
                2, // decel m/s/s
                1, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // track m
                0.5, // wheelbase m
                0.25, // front offset m
                0.6); // vcg m
    }

    public static SwerveKinodynamics forWPITest() {
        return new SwerveKinodynamics(
                1, // vel m/s
                5, // stall m/s/s
                1, // accel m/s/s
                1, // decel m/s/s
                1, // steering rate rad/s
                1, // steering accel rad/s/s
                2, // track m
                2, // track m
                2, // wheelbase m
                1, // front offset m
                1); // vcg m
    }
    //////////////////////////////////////////
    //
    // below are specific test cases. try to minimize their number

    public static SwerveKinodynamics highDecelAndCapsize() {
        return new SwerveKinodynamics(
                5, // vel m/s
                10, // stall m/s/s
                2, // accel m/s/s
                300, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // track m
                0.5, // wheelbase m
                0.25, // front offset m
                0.001); // vcg m
    }

    public static SwerveKinodynamics decelCase() {
        return new SwerveKinodynamics(
                1, // vel m/s
                10, // stall m/s/s
                1, // accel m/s/s
                10, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // track m
                0.5, // wheelbase m
                0.25, // front offset m
                0.3); // vcg m
    }

    public static SwerveKinodynamics highCapsize() {
        return new SwerveKinodynamics(
                5, // vel m/s
                20, // stall m/s/s
                10, // accel m/s/s
                10, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // track m
                0.5, // wheelbase m
                0.25, // front offset m
                0.1); // vcg m
    }

    public static SwerveKinodynamics lowCapsize() {
        return new SwerveKinodynamics(
                5, // vel m/s
                20, // stall m/s/s
                10, // accel m/s/s
                10, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // track m
                0.5, // wheelbase m
                0.25, // front offset m
                2); // vcg m (very high vcg)
    }

    public static SwerveKinodynamics limiting() {
        return new SwerveKinodynamics(
                5, // vel m/s
                30, // stall m/s/s
                10, // accel m/s/s
                10, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // track m
                0.5, // wheelbase m
                0.25, // front offset m
                0.3); // vcg m
    }

    /** Large difference in accel and decel, to make asymmetry obvious. */
    public static SwerveKinodynamics lowAccelHighDecel() {
        return new SwerveKinodynamics(
                4, // vel m/s
                10, // stall m/s/s
                1, // accel m/s/s
                10, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // track m
                0.5, // wheelbase m
                0.25, // front offset m
                0.3); // vcg m
    }

    /**
     * very high limits, should make setpoint generator do nothing.
     * 
     * note: both the wpi and 100 profiles fail to produce useful feedforward when
     * the distance is reachable in one time step, i.e. high accel and velocity
     * limits.
     */
    public static SwerveKinodynamics unlimited() {
        return new SwerveKinodynamics(
                10000, // vel m/s
                10000, // stall m/s/s
                10000, // accel m/s/s
                10000, // decel m/s/s
                10000, // steering rate rad/s
                10000, // steering accel rad/s/s
                0.5, // track m
                0.5, // track m
                0.5, // wheelbase m
                0.25, // front offset m
                0); // vcg m
    }

    private SwerveKinodynamicsFactory() {
        //
    }
}
