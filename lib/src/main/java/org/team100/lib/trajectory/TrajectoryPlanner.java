package org.team100.lib.trajectory;

import java.util.List;
import java.util.function.Function;

import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.state.ModelR3;
import org.team100.lib.trajectory.path.Path100;
import org.team100.lib.trajectory.path.PathFactory;
import org.team100.lib.trajectory.timing.ScheduleGenerator;
import org.team100.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

/**
 * Creates a trajectory in four steps:
 * 
 * 1. create a spline
 * 2. create points along the spline so that the secants between the points are
 * within the spline sample tolerance
 * 3. walk down the secant lines using the step distance
 * 4. assign timestamps to each step
 */
public class TrajectoryPlanner {
    /*
     * Maximum distance of the secant lines to the continuous spline. The resulting
     * path will have little scallops if it involves rotation. In SE(2), a constant
     * "twist" segment with rotation is a curve. If the scallops are too big, make
     * this number smaller. If the trajectories are too slow to generate, make this
     * number bigger.
     */
    private static final double SPLINE_SAMPLE_TOLERANCE_M = 0.02;
    /**
     * Maximum theta error.
     */
    private static final double SPLINE_SAMPLE_TOLERANCE_RAD = 0.2;
    /**
     * Size of steps along the path. Make this number smaller for tight curves to
     * look better. Make it bigger to make trajectories (a little) faster to
     * generate.
     */
    private static final double TRAJECTORY_STEP_M = 0.1;
    /*
     * If we try to start a trajectory while respecting initial velocity, but the
     * initial velocity is less than 0.01 m/s, just treat it as rest-to-rest.
     */
    private static final double VELOCITY_EPSILON = 1e-2;

    private final double m_splineTolerance;
    private final double m_splineRotationTolerance;
    private final double m_trajectoryStep;

    private final ScheduleGenerator m_scheduleGenerator;

    public TrajectoryPlanner(List<TimingConstraint> constraints) {
        this(SPLINE_SAMPLE_TOLERANCE_M, SPLINE_SAMPLE_TOLERANCE_RAD, TRAJECTORY_STEP_M, constraints);
    }

    public TrajectoryPlanner(
            double splineTolerance,
            double splineRotationTolerance,
            double trajectoryStep,
            List<TimingConstraint> constraints) {
        m_splineTolerance = splineTolerance;
        m_splineRotationTolerance = splineRotationTolerance;
        m_trajectoryStep = trajectoryStep;
        m_scheduleGenerator = new ScheduleGenerator(constraints);

    }

    /** A square counterclockwise starting with +x. */
    public List<Trajectory100> square(Pose2d p0) {
        Pose2d p1 = p0.plus(new Transform2d(1, 0, Rotation2d.kZero));
        Pose2d p2 = p0.plus(new Transform2d(1, 1, Rotation2d.kZero));
        Pose2d p3 = p0.plus(new Transform2d(0, 1, Rotation2d.kZero));
        return List.of(
                restToRest(p0, p1),
                restToRest(p1, p2),
                restToRest(p2, p3),
                restToRest(p3, p0));
    }

    /** Make a square that gets a reset starting point at each corner. */
    public List<Function<Pose2d, Trajectory100>> permissiveSquare() {
        return List.of(
                x -> restToRest(x, x.plus(new Transform2d(1, 0, Rotation2d.kZero))),
                x -> restToRest(x, x.plus(new Transform2d(0, 1, Rotation2d.kZero))),
                x -> restToRest(x, x.plus(new Transform2d(-1, 0, Rotation2d.kZero))),
                x -> restToRest(x, x.plus(new Transform2d(0, -1, Rotation2d.kZero))));
    }

    /** From current to x+1 */
    public Trajectory100 line(Pose2d initial) {
        return restToRest(
                initial,
                initial.plus(new Transform2d(1, 0, Rotation2d.kZero)));
    }

    public Trajectory100 restToRest(List<HolonomicPose2d> waypoints) {
        return generateTrajectory(waypoints, 0.0, 0.0);
    }

    public Trajectory100 restToRest(List<HolonomicPose2d> waypoints, List<Double> mN) {
        return generateTrajectory(waypoints, 0.0, 0.0, mN);
    }

    public Trajectory100 movingToRest(ModelR3 startState, Pose2d end) {
        return movingToMoving(startState, new ModelR3(end));
    }

    public Trajectory100 movingToMoving(ModelR3 startState, ModelR3 endState) {
        Translation2d startTranslation = startState.translation();
        GlobalVelocityR3 startVelocity = startState.velocity();

        Translation2d endTranslation = endState.translation();
        GlobalVelocityR3 endVelocity = endState.velocity();

        // should work with out this.
        if (startVelocity.norm() < VELOCITY_EPSILON && endVelocity.norm() < VELOCITY_EPSILON) {
            return restToRest(startState.pose(), endState.pose());
        }

        Translation2d full = endTranslation.minus(startTranslation);
        Rotation2d courseToGoal = full.getAngle();
        Rotation2d startingAngle = startVelocity.angle().orElse(courseToGoal);

        // use the start velocity to adjust the first magic number.
        // divide by the distance because the spline multiplies by it
        double e1 = 2.0 * startVelocity.norm() / full.getNorm();
        List<Double> magicNumbers = List.of(e1, 1.2);

        try {
            return generateTrajectory(
                    List.of(
                            new HolonomicPose2d(
                                    startTranslation,
                                    startState.rotation(),
                                    startingAngle),
                            new HolonomicPose2d(
                                    endTranslation,
                                    endState.rotation(),
                                    courseToGoal)),
                    startVelocity.norm(),
                    endVelocity.norm(),
                    magicNumbers);
        } catch (TrajectoryGenerationException e) {
            System.out.println("WARNING: Trajectory Generation Exception");
            return new Trajectory100();
        }
    }

    public Trajectory100 movingToMoving(ModelR3 startState, Rotation2d startCourse, double splineEntranceVelocity, ModelR3 endState, Rotation2d endCourse, double splineExitVelocity) {
        Translation2d startTranslation = startState.translation();
        GlobalVelocityR3 startVelocity = startState.velocity();

        Translation2d endTranslation = endState.translation();
        GlobalVelocityR3 endVelocity = endState.velocity();

        // should work with out this.
        if (startVelocity.norm() < VELOCITY_EPSILON && endVelocity.norm() < VELOCITY_EPSILON) {
            return restToRest(startState.pose(), endState.pose());
        }

        Translation2d full = endTranslation.minus(startTranslation);

        // use the start velocity to adjust the first magic number.
        // divide by the distance because the spline multiplies by it
        double e1 = 2.0 * startVelocity.norm() / full.getNorm();
        List<Double> magicNumbers = List.of(e1, 1.2);

        try {
            return generateTrajectory(
                    List.of(
                            new HolonomicPose2d(
                                    startTranslation,
                                    startState.rotation(),
                                    startCourse),
                            new HolonomicPose2d(
                                    endTranslation,
                                    endState.rotation(),
                                    endCourse)),
                    splineEntranceVelocity,
                    splineExitVelocity,
                    magicNumbers);
        } catch (TrajectoryGenerationException e) {
            System.out.println("WARNING: Trajectory Generation Exception");
            return new Trajectory100();
        }
    }

    public Trajectory100 movingToRest(ModelR3 startState, Rotation2d startCourse, double splineEntranceVelocity, Pose2d end, Rotation2d endCourse, double splineExitVelocity) {
        return movingToMoving(startState, startCourse,splineEntranceVelocity, new ModelR3(end), endCourse,splineExitVelocity);
    }

    /**
     * Produces straight lines from start to end.
     */
    public Trajectory100 restToRest(Pose2d start, Pose2d end) {
        Translation2d startTranslation = start.getTranslation();
        Translation2d endTranslation = end.getTranslation();

        Rotation2d courseToGoal = endTranslation.minus(startTranslation).getAngle();

        try {
            return restToRest(
                    List.of(
                            new HolonomicPose2d(startTranslation, start.getRotation(), courseToGoal),
                            new HolonomicPose2d(endTranslation, end.getRotation(), courseToGoal)));
        } catch (TrajectoryGenerationException e) {
            return null;
        }
    }

    /**
     * The shape of the spline accommodates the start and end velocities.
     */
    public Trajectory100 generateTrajectory(
            List<HolonomicPose2d> waypoints,
            double start_vel,
            double end_vel) {
        try {
            // Create a path from splines.
            Path100 path = PathFactory.pathFromWaypoints(
                    waypoints,
                    m_splineTolerance,
                    m_splineTolerance,
                    m_splineRotationTolerance);
            // Generate the timed trajectory.
            return m_scheduleGenerator.timeParameterizeTrajectory(
                    path,
                    m_trajectoryStep,
                    start_vel,
                    end_vel);
        } catch (IllegalArgumentException e) {
            // catches various kinds of malformed input, returns a no-op.
            // this should never actually happen.
            System.out.println("WARNING: Bad trajectory input!!");
            // print the stack trace if you want to know who is calling
            // e.printStackTrace();
            return new Trajectory100();
        }
    }

    public Trajectory100 generateTrajectory(
            List<HolonomicPose2d> waypoints,
            double start_vel,
            double end_vel,
            List<Double> mN) {
        try {
            // Create a path from splines.
            Path100 path = PathFactory.pathFromWaypoints(
                    waypoints,
                    m_splineTolerance,
                    m_splineTolerance,
                    m_splineRotationTolerance,
                    mN);
            // Generate the timed trajectory.
            return m_scheduleGenerator.timeParameterizeTrajectory(
                    path,
                    m_trajectoryStep,
                    start_vel,
                    end_vel);
        } catch (IllegalArgumentException e) {
            // catches various kinds of malformed input, returns a no-op.
            // this should never actually happen.
            System.out.println("WARNING: Bad trajectory input!!");
            e.printStackTrace();
            return new Trajectory100();
        }
    }
}
