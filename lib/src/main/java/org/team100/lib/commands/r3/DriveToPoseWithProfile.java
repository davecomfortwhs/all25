package org.team100.lib.commands.r3;

import java.util.function.Supplier;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.controller.r3.ReferenceControllerR3;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.reference.r3.ProfileReferenceR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.SubsystemR3;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Drive to a pose supplied at initialization, using a profile.
 */
public class DriveToPoseWithProfile extends MoveAndHold {
    private final SubsystemR3 m_drive;
    private final ControllerR3 m_controller;
    private final HolonomicProfile m_profile;
    private final Pose2dLogger m_log_goal;
    private final Supplier<Pose2d> m_goal;

    private ProfileReferenceR3 m_reference;
    private ReferenceControllerR3 m_referenceController;

    public DriveToPoseWithProfile(
            LoggerFactory logger,
            SubsystemR3 drive,
            ControllerR3 controller,
            HolonomicProfile profile,
            Supplier<Pose2d> goal) {
        LoggerFactory child = logger.type(this);
        m_log_goal = child.pose2dLogger(Level.TRACE, "goal");
        m_drive = drive;
        m_controller = controller;
        m_profile = profile;
        m_goal = goal;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Pose2d goal = m_goal.get();
        m_log_goal.log(() -> goal);
        m_reference = new ProfileReferenceR3(m_profile, "embark");
        m_reference.setGoal(new ModelR3(goal));
        m_referenceController = new ReferenceControllerR3(
                m_drive, m_controller, m_reference);
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_reference.end();
        m_reference = null;
        m_referenceController = null;
    }

    @Override
    public boolean isDone() {
        return m_referenceController != null && m_referenceController.isDone();
    }

    @Override
    public double toGo() {
        return (m_referenceController == null) ? 0 : m_referenceController.toGo();
    }

}
