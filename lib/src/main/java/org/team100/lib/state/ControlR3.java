package org.team100.lib.state;

import java.util.Optional;

import org.team100.lib.geometry.GlobalAccelerationR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Describes the state of three independent dimensions, each of which is
 * represented by position, velocity, and acceleration.
 * 
 * This could be used for navigation, or for other applications of rigid-body
 * transforms in 2d, e.g. planar mechanisms.
 * 
 * This type is used for control, which is why it includes acceleration
 * 
 * Do not try to use zero as an initial location; always initialize with the
 * current location.
 */
public class ControlR3 {
    private final Control100 m_x;
    private final Control100 m_y;
    private final Control100 m_theta;

    public ControlR3(Control100 x, Control100 y, Control100 theta) {
        m_x = x;
        m_y = y;
        m_theta = theta;
    }

    public ControlR3(Pose2d x, GlobalVelocityR3 v) {
        this(
                new Control100(x.getX(), v.x(), 0),
                new Control100(x.getY(), v.y(), 0),
                new Control100(x.getRotation().getRadians(), v.theta(), 0));
    }

    public ControlR3(Pose2d x, GlobalVelocityR3 v, GlobalAccelerationR3 a) {
        this(
                new Control100(x.getX(), v.x(), a.x()),
                new Control100(x.getY(), v.y(), a.y()),
                new Control100(x.getRotation().getRadians(), v.theta(), a.theta()));
    }

    public ControlR3(Pose2d x) {
        this(x, new GlobalVelocityR3(0, 0, 0));
    }

    public ControlR3(Rotation2d x) {
        this(new Pose2d(0, 0, x));
    }

    public static ControlR3 zero() {
        return new ControlR3(new Control100(), new Control100(), new Control100());
    }

    public ModelR3 model() {
        return new ModelR3(m_x.model(), m_y.model(), m_theta.model());
    }

    public ControlR3 withTheta(double theta) {
        return new ControlR3(m_x, m_y, new Control100(theta, m_theta.v(), m_theta.a()));
    }

    public ControlR3 minus(ControlR3 other) {
        return new ControlR3(x().minus(other.x()), y().minus(other.y()), theta().minus(other.theta()));
    }

    public ControlR3 plus(ControlR3 other) {
        return new ControlR3(x().plus(other.x()), y().plus(other.y()), theta().plus(other.theta()));
    }

    public boolean near(ControlR3 other, double tolerance) {
        return x().near(other.x(), tolerance)
                && y().near(other.y(), tolerance)
                && theta().near(other.theta(), tolerance);
    }

    public Pose2d pose() {
        return new Pose2d(m_x.x(), m_y.x(), rotation());
    }

    /** Translation of the pose */
    public Translation2d translation() {
        return new Translation2d(m_x.x(), m_y.x());
    }

    public Rotation2d rotation() {
        return new Rotation2d(m_theta.x());
    }

    public GlobalVelocityR3 velocity() {
        return new GlobalVelocityR3(m_x.v(), m_y.v(), m_theta.v());
    }

    /** Robot-relative speeds */
    public ChassisSpeeds chassisSpeeds() {
        return SwerveKinodynamics.toInstantaneousChassisSpeeds(velocity(), rotation());
    }

    public GlobalAccelerationR3 acceleration() {
        return new GlobalAccelerationR3(m_x.a(), m_y.a(), m_theta.a());
    }

    public Control100 x() {
        return m_x;
    }

    public Control100 y() {
        return m_y;
    }

    public Control100 theta() {
        return m_theta;
    }

    /**
     * Transform timed pose into swerve state.
     * 
     * Correctly accounts for centripetal acceleration.
     */
    public static ControlR3 fromTimedPose(TimedPose timedPose) {
        double xx = timedPose.state().getPose().getX();
        double yx = timedPose.state().getPose().getY();
        double thetax = timedPose.state().getHeading().getRadians();

        double velocityM_s = timedPose.velocityM_S();
        Optional<Rotation2d> course = timedPose.state().getCourse();
        Rotation2d motion_direction = course.isPresent() ? course.get() : Rotation2d.kZero;
        double xv = motion_direction.getCos() * velocityM_s;
        double yv = motion_direction.getSin() * velocityM_s;
        double thetav = timedPose.state().getHeadingRate() * velocityM_s;

        double accelM_s_s = timedPose.acceleration();
        double xa = motion_direction.getCos() * accelM_s_s;
        double ya = motion_direction.getSin() * accelM_s_s;
        double thetaa = timedPose.state().getHeadingRate() * accelM_s_s;

        // centripetal accel = v^2/r = v^2 * curvature
        double curvRad_M = timedPose.state().getCurvature();
        double centripetalAccelM_s_s = velocityM_s * velocityM_s * curvRad_M;
        double xCa = -1.0 * motion_direction.getSin() * centripetalAccelM_s_s;
        double yCa = motion_direction.getCos() * centripetalAccelM_s_s;

        return new ControlR3(
                new Control100(xx, xv, xa + xCa),
                new Control100(yx, yv, ya + yCa),
                new Control100(thetax, thetav, thetaa));
    }

    public String toString() {
        return "SwerveControl(" + m_x + ", " + m_y + ", " + m_theta + ")";
    }

}