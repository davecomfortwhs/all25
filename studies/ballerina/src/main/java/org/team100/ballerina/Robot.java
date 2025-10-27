package org.team100.ballerina;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.Logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot {
    private final DriverXboxControl m_controller;
    private final Turret m_turret;
    private final ManualPose m_pose;
    private final TargetDesignator m_target;
    private final Indicator m_indicator;
    private final Ball m_ball;

    public Robot() {
        Logging log = Logging.instance();
        m_controller = new DriverXboxControl(0);
        m_pose = new ManualPose(log.fieldLogger, m_controller::velocity, new Pose2d(6, 4, Rotation2d.kZero));
        m_target = new TargetDesignator(log.fieldLogger, TargetDesignator.A);
        m_turret = new Turret(log.rootLogger, log.fieldLogger, m_pose::getPose, m_target::getTarget);
        m_indicator = new Indicator(m_turret::onTarget);
        m_ball = new Ball(log.fieldLogger, m_pose::getState, m_turret::getAzimuth);

        // button 1
        new Trigger(m_controller::a).onTrue(m_target.a());
        // button 2
        new Trigger(m_controller::b).onTrue(m_target.b());
        // button 3
        new Trigger(m_controller::x).whileTrue(m_turret.aim());
        // button 4
        new Trigger(m_controller::y).whileTrue(m_ball.shoot());

        m_turret.setDefaultCommand(m_turret.stop());

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_pose.periodic();
        m_target.periodic();
        m_indicator.periodic();
        m_ball.periodic();
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void close() {
        m_indicator.close();
    }
}
