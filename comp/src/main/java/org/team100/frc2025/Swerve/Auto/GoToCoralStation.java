package org.team100.frc2025.Swerve.Auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import org.team100.lib.field.FieldConstants;
import org.team100.lib.field.FieldConstants.CoralStation;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Function to supply a rest-to-rest trajectory from the given starting point to
 * the coral station.
 */
public class GoToCoralStation implements Function<Pose2d, Trajectory100> {
    private final double m_scale;
    private final CoralStation m_station;
    private final TrajectoryPlanner m_planner;

    public GoToCoralStation(
            LoggerFactory log,
            SwerveKinodynamics kinodynamics,
            CoralStation station,
            double scale) {
        m_station = station;
        m_scale = scale;
        m_planner = new TrajectoryPlanner(
                new TimingConstraintFactory(kinodynamics).auto(log.type(this)));
    }

    @Override
    public Trajectory100 apply(Pose2d currentPose) {
        Pose2d goal = m_station.pose();
        double scaleAdjust = switch (m_station) {
            case LEFT -> m_scale;
            case RIGHT -> -1.0 * m_scale;
            default -> throw new IllegalArgumentException("invalid station");
        };

        Translation2d currTranslation = currentPose.getTranslation();
        Rotation2d courseToGoal = goal.getTranslation().minus(currTranslation).getAngle();
        Rotation2d newInitialSpline = FieldConstants.calculateDeltaSpline(
                courseToGoal, courseToGoal.rotateBy(Rotation2d.fromDegrees(-90)), scaleAdjust);

        List<HolonomicPose2d> waypoints = new ArrayList<>();
        waypoints.add(new HolonomicPose2d(currTranslation, currentPose.getRotation(), newInitialSpline));
        waypoints.add(new HolonomicPose2d(goal.getTranslation(), goal.getRotation(), courseToGoal));

        return m_planner.restToRest(waypoints);
    }

}
