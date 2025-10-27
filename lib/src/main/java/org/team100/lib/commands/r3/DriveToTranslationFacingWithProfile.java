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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Drive to a pose supplied at initialization, using a profile.
 */
public class DriveToTranslationFacingWithProfile extends MoveAndHold {
    private final SubsystemR3 m_drive;
    private final ControllerR3 m_controller;
    private final HolonomicProfile m_profile;
    private final Pose2dLogger m_log_goal;
    private final Supplier<Translation2d> m_goal;
    private final Rotation2d m_sideFacing;

    private ProfileReferenceR3 m_reference;
    private ReferenceControllerR3 m_referenceController;

    public DriveToTranslationFacingWithProfile(
            LoggerFactory logger,
            SubsystemR3 drive,
            ControllerR3 controller,
            HolonomicProfile profile,
            Supplier<Translation2d> goal,
            Rotation2d sideFacing) {
        LoggerFactory child = logger.type(this);
        m_log_goal = child.pose2dLogger(Level.TRACE, "goal");
        m_drive = drive;
        m_controller = controller;
        m_profile = profile;
        m_goal = goal;
        m_sideFacing = sideFacing;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Pose2d goal = getGoal(m_goal.get(),
                m_drive.getState().pose().getTranslation());
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

    public Pose2d getGoal(Translation2d goalTranslation, Translation2d robotTranslation) {
        return new Pose2d(goalTranslation, goalTranslation.minus(robotTranslation).getAngle().plus(m_sideFacing));
    }

    public double toGo() {
        return m_referenceController.toGo();
    }
}
