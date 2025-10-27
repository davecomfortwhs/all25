package org.team100.lib.commands.r3.test;

import java.util.Iterator;
import java.util.List;
import java.util.function.Function;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.controller.r3.ReferenceControllerR3;
import org.team100.lib.reference.r3.TrajectoryReferenceR3;
import org.team100.lib.subsystems.SubsystemR3;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Follow a list of trajectories created at initialization time, given the pose
 * at that time.
 * 
 * The only reason you would want to use a list of trajectories, rather than a
 * single trajectory with internal waypoints, is because you want the robot path
 * to have sharp corners. And the only reason you'd really want that is for
 * testing.
 */
public class DriveWithTrajectoryListFunction extends MoveAndHold {
    private final SubsystemR3 m_drive;
    private final ControllerR3 m_controller;
    private final Function<Pose2d, List<Trajectory100>> m_trajectories;
    private final TrajectoryVisualization m_viz;

    private Iterator<Trajectory100> m_trajectoryIter;
    private ReferenceControllerR3 m_referenceController;

    public DriveWithTrajectoryListFunction(
            SubsystemR3 swerve,
            ControllerR3 controller,
            Function<Pose2d, List<Trajectory100>> trajectories,
            TrajectoryVisualization viz) {
        m_drive = swerve;
        m_controller = controller;
        m_trajectories = trajectories;
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_trajectoryIter = m_trajectories.apply(m_drive.getState().pose()).iterator();
        m_referenceController = null;
    }

    @Override
    public void execute() {
        if (m_referenceController == null || m_referenceController.isDone()) {
            // get the next trajectory
            if (m_trajectoryIter.hasNext()) {
                Trajectory100 m_trajectory = m_trajectoryIter.next();
                TrajectoryReferenceR3 reference = new TrajectoryReferenceR3(m_trajectory);
                m_referenceController = new ReferenceControllerR3(
                        m_drive, m_controller, reference);
                m_viz.setViz(m_trajectory);
            } else {
                return;
            }
        }

        // now there is a trajectory to follow
        if (m_referenceController != null) {
            m_referenceController.execute();
        }
    }

    @Override
    public boolean isDone() {
        return m_referenceController == null || m_referenceController.isDone();
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
