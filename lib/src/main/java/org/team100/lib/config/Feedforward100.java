package org.team100.lib.config;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.tuning.Mutable;

/**
 * The motor feedforward model includes four constants.
 * 
 * @param kV  Velocity: ratio between unloaded speed (rev/s) and voltage, so the
 *            units are VOLT-SEC/REV. This reflects the "Back EMF" aspect of a
 *            motor: it produces a voltage proportional to speed. The value is
 *            an intrinsic property of the motor.
 * @param kA  Acceleration: ratio between acceleration (rev/s^2) and voltage, so
 *            the units are VOLT-SEC^2/REV. This reflects torque production in
 *            the motor: torque is proportional to current, which is
 *            proportional to (net) voltage. The value will depend on the
 *            inertia of the mechanism.
 * @param kD  Deceleration: like kA but when the motor is braking, i.e.
 *            acceleration is opposite to the current speed. Motors seem to
 *            decelerate much better than they accelerate -- perhaps the current
 *            limit acts differently (or not at all)?
 * @param kSS Static friction: voltage to get the motor moving from a stop, so
 *            the units are VOLTS. The value will depend on the "stickiness" of
 *            the mechanism.
 * @param kDS Dynamic friction: voltage to just barely keep the motor moving,
 *            the units are VOLTS. The value will depend on the "viscosity" of
 *            the mechanism.
 * 
 * @see https://en.wikipedia.org/wiki/Motor_constants
 * @see {@link edu.wpi.first.math.controller.SimpleMotorFeedforward} which uses
 *      a similar model, and also a discrete one which is more accurate.
 * @see {@link org.team100.lib.config.FeedforwardTest} which compares the
 *      models.
 */
public class Feedforward100 {
    private final List<Runnable> m_listeners = new ArrayList<>();

    private final Mutable kV;
    private final Mutable kA;
    private final Mutable kD;
    private final Mutable kSS;
    private final Mutable kDS;
    private final double staticFrictionSpeedLimit;

    Feedforward100(
            LoggerFactory log,
            double kV,
            double kA,
            double kD,
            double kSS,
            double kDS,
            double staticFrictionSpeedLimit) {
        this.kV = new Mutable(log, "kV", kV, this::onChange);
        this.kA = new Mutable(log, "kA", kA, this::onChange);
        this.kD = new Mutable(log, "kD", kD, this::onChange);
        this.kSS = new Mutable(log, "kSS", kSS, this::onChange);
        this.kDS = new Mutable(log, "kDS", kDS, this::onChange);
        this.staticFrictionSpeedLimit = staticFrictionSpeedLimit;
    }

    public static Feedforward100 makeNeo(LoggerFactory log) {
        return new Feedforward100(log, 0.122, 0.000, 0.000, 0.100, 0.065, 0.5);
    }

    public static Feedforward100 makeNeoArm(LoggerFactory log) {
        return new Feedforward100(log, 0.22, 1, 1, 0, 0, 0);
    }

    public static Feedforward100 makeNeo550(LoggerFactory log) {
        return new Feedforward100(log, 0.12, 0, 0, 0, .07, 0);
    }

    public static Feedforward100 makeArmPivot(LoggerFactory log) {
        return new Feedforward100(
                log,
                0.09,
                0.005,
                0.005,
                0.100,
                0.065,
                0.5);
    }

    public static Feedforward100 makeClimber(LoggerFactory log) {
        return new Feedforward100(
                log,
                0.14,
                0.05,
                0.05,
                0.100,
                0.1,
                0.0);
    }

    public static Feedforward100 zero(LoggerFactory log) {
        return new Feedforward100(log, 0, 0, 0, 0, 0, 0);
    }

    public static Feedforward100 makeNeoVortex(LoggerFactory log) {
        return new Feedforward100(log, 0.122, 0.000, 0.000, 0.100, 0.065, 0.5);
    }

    public static Feedforward100 makeWCPSwerveTurningFalcon(LoggerFactory log) {
        return new Feedforward100(log, 0.110, 0.000, 0.000, 0.180, 0.010, 0.5);
    }

    /**
     * 9/24/04
     * Voltage feedforward for steering motors in air.
     * Tuned in air, not on carpet, so probably the velocity number is too low.
     */
    public static Feedforward100 makeWCPSwerveTurningFalcon6(LoggerFactory log) {
        return new Feedforward100(log, 0.150, 0.010, 0.010, 0.080, 0.100, 0.5);
    }

    public static Feedforward100 makeKrakenTurret(LoggerFactory log) {
        return new Feedforward100(log, 0.150, 0.010, 0.010, 0.080, 0.100, 0.5);
    }

    public static Feedforward100 makeWCPSwerveDriveFalcon(LoggerFactory log) {
        return new Feedforward100(log, 0.110, 0.000, 0.000, 0.375, 0.270, 0.5);
    }

    public static Feedforward100 makeWCPSwerveDriveFalcon6(LoggerFactory log) {
        return new Feedforward100(log, 0.13, 0.017, 0.017, 0, 0.26, 0.06);
    }

    public static Feedforward100 makeWCPSwerveDriveKraken6(LoggerFactory log) {
        return new Feedforward100(log, 0.13, 0.022, 0.007, 0.26, 0.26, 0.06); // WAS 0.115
    }

    public static Feedforward100 makeAMSwerveDriveFalcon6(LoggerFactory log) {
        return new Feedforward100(log, 0.110, 0.000, 0.000, 0.180, 0.010, 0.1);
    }

    public static Feedforward100 makeSimple(LoggerFactory log) {
        return new Feedforward100(log, 0.100, 0.100, 0.100, 0.100, 0.100, 0.1);
    }

    public static Feedforward100 makeShooterFalcon6(LoggerFactory log) {
        return new Feedforward100(log, 0.110, 0.000, 0.000, 0.000, 0.900, 0.1);
    }

    public static Feedforward100 makeKraken6Elevator(LoggerFactory log) {
        return new Feedforward100(log, 0.135, 0.005, 0.005, 0.005, 0.005, 0.1);
        // return new Feedforward100(0, 0, 0, 0, 0);

    }

    public static Feedforward100 makeKraken6Wrist(LoggerFactory log) {
        return new Feedforward100(log, 0.137, 0.0085, 0.002, 0.11, 0.11, 0.1);
        // return new Feedforward100(log, 0.055, 0.025, 0.015, 0.005, 0.005, 0.1);

    }

    public static Feedforward100 makeKraken6WristWithLowerKd(LoggerFactory log) {
        return new Feedforward100(log, 0.14, 0.013, 0.005, 0.2, 0.23, 0.1);
        // return new Feedforward100(0, 0, 0, 0, 0);
    }

    public static Feedforward100 makeKrakenClimberIntake(LoggerFactory log) {
        return new Feedforward100(log, 0.13, 0.022, 0.007, 0.26, 0.26, 0.06);
    }

    public double velocityFFVolts(double motorRev_S) {
        return kV.getAsDouble() * motorRev_S;
    }

    /**
     * Uses kA when speed and accel are in the same direction.
     * Uses kD when speed and accel are opposite.
     * 
     * @param motorRev_S   setpoint speed
     * @param motorRev_S_S setpoint acceleration
     */
    public double accelFFVolts(double motorRev_S, double motorRev_S_S) {
        if (motorRev_S >= 0) {
            // moving forward
            if (motorRev_S_S >= 0) {
                // faster
                return kA.getAsDouble() * motorRev_S_S;
            } else {
                // slower
                return kD.getAsDouble() * motorRev_S_S;
            }
        } else {
            // moving backward
            if (motorRev_S_S < 0) {
                // faster
                return kA.getAsDouble() * motorRev_S_S;
            } else {
                // slower
                return kD.getAsDouble() * motorRev_S_S;
            }
        }
    }

    /**
     * @param motorRev_S setpoint speed
     */
    public double frictionFFVolts(double motorRev_S) {
        double direction = Math.signum(motorRev_S);
        if (Math.abs(motorRev_S) < staticFrictionSpeedLimit) {
            return kSS.getAsDouble() * direction;
        }
        return kDS.getAsDouble() * direction;
    }

    public static Feedforward100 makeTest1(LoggerFactory log) {
        return new Feedforward100(log, 0.3, 0.000, 0.000, 0.000, 0.100, 3.5);
    }

    /////////////////////////////////////////////////////////////////////////

    public void register(Runnable listener) {
        m_listeners.add(listener);
    }

    private void onChange(double ignored) {
        m_listeners.stream().forEach(r -> r.run());
    }
}
