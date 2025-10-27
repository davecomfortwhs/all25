package org.team100.lib.commands.swerve.manual;

import java.util.function.Supplier;

import org.team100.lib.controller.r1.Feedback100;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.geometry.TargetUtil;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.incremental.IncrementalProfile;
import org.team100.lib.profile.incremental.TrapezoidIncrementalProfile;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.state.ModelR3;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Manual cartesian control, with rotational control based on a target position.
 * 
 * This is useful for shooting solutions, or for keeping the camera pointed at
 * something.
 * 
 * Rotation uses a profile, velocity feedforward, and positional feedback.
 * 
 * The targeting solution is based on bearing alone, so it won't work if the
 * robot or target is moving. That effect can be compensated, though.
 */
public class ManualWithTargetLock implements FieldRelativeDriver {
    /**
     * Relative rotational speed. Use a moderate value to trade rotation for
     * translation
     */
    private static final double ROTATION_SPEED = 0.5;

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Supplier<Translation2d> m_target;
    private final Feedback100 m_thetaController;
    private final IncrementalProfile m_profile;

    private final DoubleLogger m_log_apparent_motion;
    private final FieldLogger.Log m_field_log;

    private Control100 m_thetaSetpoint;

    public ManualWithTargetLock(
            FieldLogger.Log fieldLogger,
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            Supplier<Translation2d> target,
            Feedback100 thetaController) {
        m_field_log = fieldLogger;
        LoggerFactory child = parent.type(this);
        m_swerveKinodynamics = swerveKinodynamics;
        m_target = target;
        m_thetaController = thetaController;
        m_profile = new TrapezoidIncrementalProfile(
                swerveKinodynamics.getMaxAngleSpeedRad_S() * ROTATION_SPEED,
                swerveKinodynamics.getMaxAngleAccelRad_S2() * ROTATION_SPEED,
                0.01);
        m_log_apparent_motion = child.doubleLogger(Level.TRACE, "apparent motion");
    }

    @Override
    public void reset(ModelR3 state) {
        m_thetaSetpoint = state.theta().control();
        m_thetaController.reset();
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds.
     * 
     * This uses the current-instant setpoint to calculate feedback error.
     * 
     * It uses the next-time-step setpoint for feedforward.
     * 
     * @param state from the drivetrain
     * @param input control units [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    @Override
    public GlobalVelocityR3 apply(
            final ModelR3 state,
            final Velocity input) {

        //
        // feedback is based on the previous setpoint.
        //

        final double thetaFB = m_thetaController.calculate(state.theta(), m_thetaSetpoint.model());

        //
        // update the setpoint for the next time step
        //

        // the goal omega should match the target's apparent motion
        final Translation2d target = m_target.get();
        final double targetMotion = TargetUtil.targetMotion(state, target);

        final Translation2d currentTranslation = state.pose().getTranslation();
        Rotation2d absoluteBearing = TargetUtil.bearing(currentTranslation, target);

        final double yaw = state.theta().x();
        absoluteBearing = new Rotation2d(
                Math100.getMinDistance(yaw, absoluteBearing.getRadians()));

        final Model100 goal = new Model100(absoluteBearing.getRadians(), targetMotion);

        // make sure the setpoint uses the modulus close to the measurement.
        m_thetaSetpoint = new Control100(
                Math100.getMinDistance(yaw, m_thetaSetpoint.x()),
                m_thetaSetpoint.v());
        m_thetaSetpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_thetaSetpoint, goal);

        // feedforward is for the next time step
        final double thetaFF = m_thetaSetpoint.v();

        final double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        final GlobalVelocityR3 scaledInput = getScaledInput(input);

        final GlobalVelocityR3 twistWithLockM_S = new GlobalVelocityR3(
                scaledInput.x(), scaledInput.y(), omega);

        m_log_apparent_motion.log(() -> targetMotion);
        m_field_log.m_log_target.log(() -> new double[] {
                target.getX(),
                target.getY(),
                0 });

        return twistWithLockM_S;
    }

    private GlobalVelocityR3 getScaledInput(Velocity input) {
        // clip the input to the unit circle
        Velocity clipped = input.clip(1.0);
        // this is user input scaled to m/s and rad/s
        GlobalVelocityR3 scaledInput = FieldRelativeDriver.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        return scaledInput;
    }
}
