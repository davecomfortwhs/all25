package org.team100.lib.motion.swerve.module;

import org.team100.lib.controller.r1.Feedback100;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.encoder.CombinedRotaryPositionSensor;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.sim.SimulatedBareEncoder;
import org.team100.lib.encoder.sim.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motion.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.incremental.IncrementalProfile;
import org.team100.lib.reference.r1.IncrementalProfileReferenceR1;

/**
 * Uses simulated position sensors, must be used with clock control (e.g.
 * {@link Timeless}).
 */
public class SimulatedSwerveModule100 extends SwerveModule100 {

    private static final double DRIVE_GEAR_RATIO = 5.5;
    private static final double WHEEL_DIAMETER_M = 0.1;

    public static SimulatedSwerveModule100 get(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics) {
        LinearVelocityServo driveServo = simulatedDriveServo(
                parent.name("Drive"));
        AngularPositionServo turningServo = simulatedTurningServo(
                parent.name("Turning"),
                kinodynamics);
        return new SimulatedSwerveModule100(driveServo, turningServo);
    }

    /**
     * The simulated outboard servo instantaneously obeys position input
     */
    public static SimulatedSwerveModule100 withInstantaneousSteering(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics) {
        LinearVelocityServo driveServo = simulatedDriveServo(
                parent.name("Drive"));
        AngularPositionServo turningServo = simulatedOutboardTurningServo(
                parent.name("Turning"),
                kinodynamics);
        return new SimulatedSwerveModule100(driveServo, turningServo);
    }

    private static LinearVelocityServo simulatedDriveServo(LoggerFactory parent) {
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(parent, driveMotor);
        LinearMechanism mech = new LinearMechanism(parent,
                driveMotor,
                encoder,
                DRIVE_GEAR_RATIO,
                WHEEL_DIAMETER_M,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        return new OutboardLinearVelocityServo(parent, mech);
    }

    /**
     * Uses simulated position sensors, must be used with clock control (e.g.
     * {@link Timeless}).
     */
    private static AngularPositionServo simulatedTurningServo(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics) {
        // simulated turning motor free speed is 20 rad/s
        SimulatedBareMotor turningMotor = new SimulatedBareMotor(parent, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(parent, turningMotor);
        SimulatedRotaryPositionSensor turningSensor = new SimulatedRotaryPositionSensor(
                parent, encoder, 1);
        RotaryMechanism turningMech = new RotaryMechanism(
                parent, turningMotor, turningSensor, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        Feedback100 turningPositionFeedback = new PIDFeedback(
                parent,
                10, // kP .. was 20
                0, // kI
                0, // kD
                false,
                0.05, // note low tolerance
                1);
        IncrementalProfile profile = kinodynamics.getSteeringProfile();
        // without a profile, there's no velocity feedforward. Hm.
        IncrementalProfileReferenceR1 ref = new IncrementalProfileReferenceR1(profile, 0.05, 0.05);
        OnboardAngularPositionServo turningServo = new OnboardAngularPositionServo(
                parent,
                turningMech,
                ref,
                turningPositionFeedback);
        turningServo.reset();
        return turningServo;
    }

    /**
     * Simulates direct (instant) positional control, which avoids the problem
     * where the steering profiles don't produce feedforwards for high speeds and
     * small errors.
     */
    private static AngularPositionServo simulatedOutboardTurningServo(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics) {
        // simulated turning motor free speed is 20 rad/s
        SimulatedBareMotor motor = new SimulatedBareMotor(parent, 600);
        SimulatedBareEncoder encoder = new SimulatedBareEncoder(parent, motor);
        SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                parent, encoder, 1);

        ProxyRotaryPositionSensor proxy = new ProxyRotaryPositionSensor(encoder, 1);
        CombinedRotaryPositionSensor combinedEncoder = new CombinedRotaryPositionSensor(
                parent, sensor, proxy);

        RotaryMechanism turningMech = new RotaryMechanism(
                parent, motor, combinedEncoder, 1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        IncrementalProfile profile = kinodynamics.getSteeringProfile();
        IncrementalProfileReferenceR1 ref = new IncrementalProfileReferenceR1(profile, 0.05, 0.05);

        OutboardAngularPositionServo turningServo = new OutboardAngularPositionServo(
                parent, turningMech, ref);
        turningServo.reset();
        return turningServo;
    }

    private SimulatedSwerveModule100(
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo) {
        // primary is 2:1 so final is whatever is left.
        super(driveServo, turningServo, WHEEL_DIAMETER_M, DRIVE_GEAR_RATIO / 2);
        //
    }
}
