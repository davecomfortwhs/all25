package org.team100.lib.commands.r3.test;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.controller.r3.ReferenceControllerR3;
import org.team100.lib.reference.r3.TrajectoryReferenceR3;
import org.team100.lib.subsystems.SubsystemR3;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

/**
 * Follow a single trajectory.
 * 
 * Because the whole trajectory is fixed, this command can only be used if the
 * robot happens to be in that spot -- and that is often hard to guarantee.
 * 
 * So this is really just for testing.
 */
public class DriveWithTrajectory extends MoveAndHold {
    private final SubsystemR3 m_drive;
    private final ControllerR3 m_controller;
    private final Trajectory100 m_trajectory;
    private final TrajectoryVisualization m_viz;

    private ReferenceControllerR3 m_referenceController;

    public DriveWithTrajectory(
            SubsystemR3 drive,
            ControllerR3 controller,
            Trajectory100 trajectory,
            TrajectoryVisualization viz) {
        m_drive = drive;
        m_controller = controller;
        m_trajectory = trajectory;
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        TrajectoryReferenceR3 reference = new TrajectoryReferenceR3(m_trajectory);
        m_referenceController = new ReferenceControllerR3(
                m_drive, m_controller, reference);
        m_viz.setViz(m_trajectory);
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    @Override
    public boolean isDone() {
        return m_referenceController.isDone();
    }

    @Override
    public double toGo() {
        return (m_referenceController == null) ? 0 : m_referenceController.toGo();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_viz.clear();
    }
}
