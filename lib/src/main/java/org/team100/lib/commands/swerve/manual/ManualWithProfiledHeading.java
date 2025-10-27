package org.team100.lib.commands.swerve.manual;

import java.util.function.Supplier;

import org.team100.lib.controller.r1.Feedback100;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.incremental.TrapezoidIncrementalProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.state.ModelR3;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Function that supports manual cartesian control, and both manual and locked
 * rotational control.
 * 
 * Rotation uses a profile, velocity feedforward, and positional feedback.
 * 
 * The profile speed and accel limits are inversely proportional to the
 * current robot speed. The reason is that, when moving near their maximum
 * speeds, the drive motors have very little torque available. If you try to
 * rotate faster, the robot just won't keep up with the profile, it will result
 * in controller error and overshoot. To the driver, the variable profile means
 * that the robot won't rotate much when you're driving fast, and it will
 * "catch up" when you slow down. This may, or may not, match the driver's
 * expectations.
 */
public class ManualWithProfiledHeading implements FieldRelativeDriver {
    // don't try to go full speed
    private static final double PROFILE_SPEED = 0.5;
    // accelerate gently to avoid upset
    private static final double PROFILE_ACCEL = 0.1;
    private final SwerveKinodynamics m_swerveKinodynamics;
    /** Absolute input supplier, null if free */
    private final Supplier<Rotation2d> m_desiredRotation;
    private final HeadingLatch m_latch;
    private final Feedback100 m_thetaFeedback;

    // LOGGERS
    private final BooleanLogger m_log_snap_mode;
    private final DoubleLogger m_log_max_speed;
    private final DoubleLogger m_log_max_accel;
    private final DoubleLogger m_log_goal_theta;
    private final Control100Logger m_log_setpoint_theta;
    private final DoubleLogger m_log_theta_FF;
    private final DoubleLogger m_log_theta_FB;
    private final DoubleLogger m_log_output_omega;

    // package private for testing
    Rotation2d m_goal = null;
    Control100 m_thetaSetpoint = null;

    /**
     * 
     * @param parent
     * @param swerveKinodynamics
     * @param desiredRotation    absolute input supplier, null if free. usually
     *                           POV-derived.
     * @param thetaController
     * @param omegaController
     */
    public ManualWithProfiledHeading(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Rotation2d> desiredRotation,
            Feedback100 thetaController) {
        LoggerFactory child = parent.type(this);
        m_swerveKinodynamics = swerveKinodynamics;
        m_desiredRotation = desiredRotation;
        m_thetaFeedback = thetaController;
        m_latch = new HeadingLatch();
        m_log_snap_mode = child.booleanLogger(Level.TRACE, "snap mode");
        m_log_max_speed = child.doubleLogger(Level.TRACE, "maxSpeedRad_S");
        m_log_max_accel = child.doubleLogger(Level.TRACE, "maxAccelRad_S2");
        m_log_goal_theta = child.doubleLogger(Level.TRACE, "goal/theta");
        m_log_setpoint_theta = child.control100Logger(Level.TRACE, "setpoint/theta");
        m_log_theta_FF = child.doubleLogger(Level.TRACE, "thetaFF");
        m_log_theta_FB = child.doubleLogger(Level.TRACE, "thetaFB");
        m_log_output_omega = child.doubleLogger(Level.TRACE, "output/omega");
    }

    @Override
    public void reset(ModelR3 state) {
        m_thetaSetpoint = state.theta().control();
        m_goal = null;
        m_latch.unlatch();
        m_thetaFeedback.reset();
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds.
     * 
     * If you touch the POV and not the twist rotation, it remembers the POV. if you
     * use the twist rotation, it forgets and just uses that.
     * 
     * Desaturation prefers the rotational profile completely in the snap case, and
     * normally in the non-snap case.
     * 
     * @param state    current drivetrain state from the pose estimator
     * @param twist1_1 control units, [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    @Override
    public GlobalVelocityR3 apply(
            final ModelR3 state,
            final Velocity twist1_1) {
        final GlobalVelocityR3 control = clipAndScale(twist1_1);

        final double currentVelocity = state.velocity().norm();

        final TrapezoidIncrementalProfile m_profile = makeProfile(currentVelocity);

        final Rotation2d pov = m_desiredRotation.get();

        m_goal = m_latch.latchedRotation(
                m_profile.getMaxAcceleration(),
                state.theta(),
                pov,
                control.theta());

        if (m_goal == null) {
            // we're not in snap mode, so it's pure manual
            // in this case there is no setpoint
            m_thetaSetpoint = null;
            m_log_snap_mode.log(() -> false);
            return control;
        }

        // if this is the first run since the latch, then the setpoint should be
        // whatever the measurement is
        if (m_thetaSetpoint == null) {
            m_thetaSetpoint = state.theta().control();
        }

        final double thetaFB = m_thetaFeedback.calculate(state.theta(), m_thetaSetpoint.model());

        //
        // feedforward uses the new setpoint
        //

        final double yawMeasurement = state.theta().x();
        // take the short path
        m_goal = new Rotation2d(
                Math100.getMinDistance(yawMeasurement, m_goal.getRadians()));
        // in snap mode we take dx and dy from the user, and use the profile for dtheta.
        // the omega goal in snap mode is always zero.
        final Model100 goalState = new Model100(m_goal.getRadians(), 0);

        // use the modulus closest to the measurement
        m_thetaSetpoint = new Control100(
                Math100.getMinDistance(yawMeasurement, m_thetaSetpoint.x()),
                m_thetaSetpoint.v());
        m_thetaSetpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_thetaSetpoint, goalState);

        // the snap overrides the user input for omega.
        final double thetaFF = m_thetaSetpoint.v();

        final double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        GlobalVelocityR3 twistWithSnapM_S = new GlobalVelocityR3(control.x(), control.y(), omega);

        m_log_snap_mode.log(() -> true);
        m_log_goal_theta.log(m_goal::getRadians);
        m_log_setpoint_theta.log(() -> m_thetaSetpoint);
        m_log_theta_FF.log(() -> thetaFF);
        m_log_theta_FB.log(() -> thetaFB);
        m_log_output_omega.log(() -> omega);

        return twistWithSnapM_S;
    }

    public GlobalVelocityR3 clipAndScale(Velocity twist1_1) {
        // clip the input to the unit circle
        final Velocity clipped = twist1_1.clip(1.0);

        // scale to max in both translation and rotation
        return FieldRelativeDriver.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
    }

    /**
     * Note that the max speed and accel are inversely proportional to the current
     * velocity.
     */
    public TrapezoidIncrementalProfile makeProfile(double currentVelocity) {
        // fraction of the maximum speed
        final double xyRatio = Math.min(1, currentVelocity / m_swerveKinodynamics.getMaxDriveVelocityM_S());
        // fraction left for rotation
        final double oRatio = 1 - xyRatio;
        // add a little bit of default speed
        final double rotationSpeed = Math.max(0.1, oRatio);

        final double maxSpeedRad_S = m_swerveKinodynamics.getMaxAngleSpeedRad_S() * rotationSpeed * PROFILE_SPEED;

        final double maxAccelRad_S2 = m_swerveKinodynamics.getMaxAngleAccelRad_S2() * rotationSpeed * PROFILE_ACCEL;

        m_log_max_speed.log(() -> maxSpeedRad_S);
        m_log_max_accel.log(() -> maxAccelRad_S2);

        return new TrapezoidIncrementalProfile(
                maxSpeedRad_S,
                maxAccelRad_S2,
                0.01);
    }
}
