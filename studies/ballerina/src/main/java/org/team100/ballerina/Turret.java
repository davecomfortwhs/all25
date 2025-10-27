package org.team100.ballerina;

import java.util.function.Supplier;

import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.incremental.IncrementalProfile;
import org.team100.lib.profile.incremental.TrapezoidIncrementalProfile;
import org.team100.lib.reference.r1.IncrementalProfileReferenceR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Attempts to maintain aim. Demonstrates "spotting". */
public class Turret extends SubsystemBase {
    private static final double GEAR_RATIO = 100;
    private static final double MIN_POSITION = -3;
    private static final double MAX_POSITION = 3;
    private final DoubleArrayLogger m_log_field_turret;
    private final Supplier<Pose2d> m_pose;
    private final Supplier<Translation2d> m_target;
    private final AngularPositionServo m_pivot;
    private boolean m_aiming;

    /**
     * @param parent Log
     * @param field  Field2d log
     * @param pose   Current robot pose, from the pose estimator.
     * @param target From the target designator; can change at any time.
     */
    public Turret(
            LoggerFactory parent,
            LoggerFactory field,
            Supplier<Pose2d> pose,
            Supplier<Translation2d> target) {
        LoggerFactory log = parent.type(this);
        m_log_field_turret = field.doubleArrayLogger(Level.COMP, "turret");
        m_pose = pose;
        m_target = target;
        IncrementalProfile profile = new TrapezoidIncrementalProfile(5, 10, 0.05);
        ProfileReferenceR1 ref = new IncrementalProfileReferenceR1(profile, 0.05, 0.05);
        PIDFeedback feedback = new PIDFeedback(log, 5, 0, 0, false, 0.05, 0.1);
        SimulatedBareMotor motor = new SimulatedBareMotor(log, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(log, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                log, encoder, GEAR_RATIO);
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, sensor, GEAR_RATIO, MIN_POSITION, MAX_POSITION);
        m_pivot = new OnboardAngularPositionServo(
                log, mech, ref, feedback);
        m_pivot.reset();
        m_aiming = false;
    }

    public boolean onTarget() {
        return m_aiming && m_pivot.atGoal();
    }

    /** Absolute turret rotation */
    public Rotation2d getAzimuth() {
        Rotation2d relative = new Rotation2d(m_pivot.getWrappedPositionRad());
        return m_pose.get().getRotation().plus(relative);
    }

    private void moveToAim() {
        m_aiming = true;
        Pose2d pose = m_pose.get();
        Translation2d target = m_target.get();
        Rotation2d absoluteBearing = target.minus(pose.getTranslation()).getAngle();
        Rotation2d relativeBearing = absoluteBearing.minus(pose.getRotation());
        m_pivot.setPositionProfiled(relativeBearing.getRadians(), 0);
    }

    private void stopAiming() {
        m_aiming = false;
        m_pivot.stop();
    }

    ////////////////////////////////////////////////////////
    ///
    /// COMMANDS

    public Command aim() {
        return run(this::moveToAim);
    }

    public Command stop() {
        return run(this::stopAiming);
    }

    @Override
    public void periodic() {
        m_pivot.periodic();
        m_log_field_turret.log(this::poseArray);
    }

    private double[] poseArray() {
        Pose2d pose = m_pose.get();
        return new double[] {
                pose.getX(),
                pose.getY(),
                pose.getRotation().plus(
                        new Rotation2d(m_pivot.getWrappedPositionRad()))
                        .getDegrees() };
    }

}
