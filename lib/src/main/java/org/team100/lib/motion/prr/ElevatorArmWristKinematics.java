package org.team100.lib.motion.prr;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Kinematics coordinates are as follows:
 * 
 * x axis is pointing up
 * y axis is pointing to the left
 * 
 * shoulder height is along x, relative to the origin
 * shoulder angle is relative to straight-up, positive counterclockwise
 * wrist angle is relative to the arm, positive counterclockwise
 * 
 * I chose these coordinates so that none of the angles ever transit the
 * pi/-pi boundary, which makes control simpler.
 * 
 * To avoid discontinuity, this now always uses the "reach up" direction, never
 * the "reach down" direction.
 */

public class ElevatorArmWristKinematics {
    private final double m_armLength;
    private final double m_manipulatorLength;

    public ElevatorArmWristKinematics(double armLength, double manipulatorLength) {
        m_armLength = armLength;
        m_manipulatorLength = manipulatorLength;
    }

    /**
     * Pose relative to the mech origin; remember to transform it to robot
     * coordinates.
     */
    public Pose2d forward(Config config) {
        double q1 = config.shoulderHeight();
        double q2 = config.shoulderAngle();
        double q3 = config.wristAngle();
        double c2 = Math.cos(q2);
        double s2 = Math.sin(q2);
        double c23 = Math.cos(q2 + q3);
        double s23 = Math.sin(q2 + q3);
        double l2 = m_armLength;
        double l3 = m_manipulatorLength;
        double x = q1 + l2 * c2 + l3 * c23;
        double y = l2 * s2 + l3 * s23;
        double r = q2 + q3;
        return new Pose2d(x, y, new Rotation2d(r));
    }

    /** Distance from shoulder pivot to wrist pivot. */
    public double armX(Translation2d wrist) {
        double d = m_armLength * m_armLength - wrist.getY() * wrist.getY();
        if (d < 0) {
            // Arm is horizontal.
            return 0;
        }
        return Math.sqrt(d);
    }

    public Config inverse(Pose2d pose) {
        /** Translation from wrist axis to tool point. */
        Translation2d wristToTip = new Translation2d(m_manipulatorLength, pose.getRotation());

        /** Location of wrist axis. */
        Translation2d wrist = pose.getTranslation().minus(wristToTip);

        /** Translation from shoulder axis to wrist axis. */
        Translation2d shoulderToWrist = new Translation2d(armX(wrist), wrist.getY());

        /** Location of shoulder axis. */
        Translation2d shoulder = wrist.minus(shoulderToWrist);

        /** Shoulder angle from vertical to arm. */
        Rotation2d shoulderAngle = shoulderToWrist.getAngle();

        /** Wrist angle from arm to tool point. */
        Rotation2d wristAngle = pose.getRotation().minus(shoulderToWrist.getAngle());

        return new Config(
                shoulder.getX(),
                shoulderAngle.getRadians(),
                wristAngle.getRadians());
    }

    public double getArmLength() {
        return m_armLength;
    }

    public double getManipulatorLength() {
        return m_manipulatorLength;
    }
}
