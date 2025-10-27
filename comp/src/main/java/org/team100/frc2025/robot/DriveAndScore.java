package org.team100.frc2025.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.commands.r3.DriveToPoseWithProfile;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.controller.r3.FullStateControllerR3;
import org.team100.lib.field.FieldConstants;
import org.team100.lib.field.FieldConstants.ReefPoint;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Use a profile to drive to the specified ReefPoint, then score at the
 * specified ScoringLevel, and finally return the mechanism to home.
 */
public class DriveAndScore {
    /** While driving to scoring tag, pay attention only to very close tags. */
    private static final double HEED_RADIUS_M = 3;

    private final LoggerFactory m_logger;
    private final Machinery m_machinery;
    private final HolonomicProfile m_autoProfile;
    private final FullStateControllerR3 m_autoController;

    public DriveAndScore(
            LoggerFactory logger,
            Machinery machinery,
            HolonomicProfile autoProfile,
            FullStateControllerR3 autoController) {
        m_logger = logger;
        m_machinery = machinery;
        m_autoProfile = autoProfile;
        m_autoController = autoController;
    }

    public Command get(ScoringLevel level, ReefPoint point) {
        MoveAndHold toReef = new DriveToPoseWithProfile(
                m_logger, m_machinery.m_drive,
                m_autoController, m_autoProfile,
                () -> FieldConstants.makeGoal(level, point));
        MoveAndHold toL4 = m_machinery.m_mech.homeToL4();
        Command eject = m_machinery.m_manipulator.centerEject().withTimeout(0.5);
        return sequence(
                parallel(
                        runOnce(() -> m_machinery.m_localizer.setHeedRadiusM(HEED_RADIUS_M)),
                        toReef,
                        waitUntil(toReef::isDone).andThen(toL4),
                        waitUntil(() -> toReef.isDone() && toL4.isDone())
                                .andThen(eject))
                        .until(() -> (toReef.isDone() && toL4.isDone() && eject.isFinished())),
                m_machinery.m_mech.l4ToHome());
    }
}
