package org.team100.ballerina;

import java.util.function.Supplier;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GlobalVelocityR2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Simulated projectile. */
public class Ball {
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;
    private static final double SPEED_M_S = 1;
    private final DoubleArrayLogger m_log_field_ball;
    private final Supplier<ModelR3> m_robot;
    private final Supplier<Rotation2d> m_azimuth;

    // null when contained in robot.
    private Translation2d m_location;
    private GlobalVelocityR2 m_velocity;

    public Ball(
            LoggerFactory field,
            Supplier<ModelR3> robot,
            Supplier<Rotation2d> azimuth) {
        m_log_field_ball = field.doubleArrayLogger(Level.COMP, "ball");
        m_robot = robot;
        m_azimuth = azimuth;
    }

    private void launch() {
        // velocity due only to the gun
        GlobalVelocityR2 v = GlobalVelocityR2.fromPolar(m_azimuth.get(), SPEED_M_S);
        // velocity due to robot translation
        GlobalVelocityR2 mv = GlobalVelocityR2.fromSe2(m_robot.get().velocity());
        // initial position
        m_location = m_robot.get().pose().getTranslation();
        // initial velocity
        m_velocity = v.plus(mv);
    }

    private void fly() {
        // evolve state with zero acceleration
        m_location = m_velocity.integrate(m_location, DT);
    }

    private void reset() {
        m_location = null;
    }

    /** Shoot the ball and continue its path as long as the command runs. */
    public Command shoot() {
        return Commands.startRun(this::launch, this::fly)
                .finallyDo(this::reset);
    }

    public void periodic() {
        m_log_field_ball.log(this::poseArray);
    }

    private double[] poseArray() {
        Translation2d t = m_location == null ? m_robot.get().translation() : m_location;
        return new double[] { t.getX(), t.getY(), 0 };
    }
}
