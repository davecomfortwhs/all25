package org.team100.lib.motion.swerve.kinodynamics;

import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;
import org.team100.lib.motion.swerve.module.state.SwerveModuleDelta;
import org.team100.lib.motion.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.motion.swerve.module.state.SwerveModulePositions;
import org.team100.lib.motion.swerve.module.state.SwerveModuleState100;
import org.team100.lib.motion.swerve.module.state.SwerveModuleStates;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Helper class that converts between chassis state and module state.
 * 
 * Note: forward kinematics is never more accurate than the gyro and we
 * absolutely cannot operate without a functional gyro, so we should use the
 * gyro instead. see https://github.com/Team100/all24/issues/350
 */
public class SwerveDriveKinematics100 {
    private final int m_numModules;
    private final Translation2d[] m_moduleLocations;

    /**
     * this (2n x 3) matrix looks something like
     * 
     * <pre>
     * 1   0  -y1
     * 0   1   x1
     * 1   0  -y2
     * 0   1   x2
     * ...
     * </pre>
     * 
     * the chassis speed vector is something like
     * 
     * <pre>
     * vx  vy  omega
     * </pre>
     * 
     * when these are multiplied, the result is
     * 
     * <pre>
     * vx - y1 omega = vx1
     * vy + x1 omega = vy1
     * vx - y2 omega = vx2
     * vy + x2 omega = xy2
     * ...
     * </pre>
     */

    final SimpleMatrix m_inverseKinematics;
    /**
     * like this:
     * this (3 x 2n) matrix is the pseudo-inverse of above, which ends up something
     * 
     * <pre>
     *  0.25 0.00 0.25 0.00 ...
     *  0.00 0.25 0.00 0.25 ...
     * -1.00 1.00 1.00 1.00 ...
     * </pre>
     * 
     * the last row depends on the drive dimensions.
     * 
     * the module state vector is something like
     * 
     * <pre>
     * vx1
     * vy1
     * vx2
     * vy2
     * ...
     * </pre>
     * 
     * so when multiplied, the result is
     * 
     * <pre>
     * mean(vx)
     * mean(vy)
     * some combination depending on dimensions
     * </pre>
     */
    final SimpleMatrix m_forwardKinematics;

    /**
     * array order:
     * 
     * frontLeft
     * frontRight
     * rearLeft
     * rearRight
     * 
     * @param moduleTranslationsM relative to the center of rotation
     */
    public SwerveDriveKinematics100(Translation2d... moduleTranslationsM) {
        checkModuleCount(moduleTranslationsM);
        m_numModules = moduleTranslationsM.length;
        m_moduleLocations = Arrays.copyOf(moduleTranslationsM, m_numModules);
        m_inverseKinematics = inverseMatrix(m_moduleLocations);
        m_forwardKinematics = m_inverseKinematics.pseudoInverse();
    }

    /**
     * INVERSE: chassis speeds -> module states
     * 
     * The resulting module state speeds are always positive.
     * 
     * States may include empty angles for motionless wheels.
     * 
     * Angles are otherwise always within [-pi, pi].
     */
    public SwerveModuleStates toSwerveModuleStates(DiscreteSpeed speed) {
        // [vx; vy; omega] (3 x 1)
        SimpleMatrix chassisSpeedsVector = chassisSpeeds2Vector(speed);
        // [v cos; v sin; ...] (2n x 1)
        return statesFromVector(chassisSpeedsVector);
    }

    /**
     * INVERSE: twist -> module position deltas
     * 
     * This assumes the wheel paths are geodesics; steering does not change.
     * 
     * Only used in tests for now.
     * 
     * States may include empty angles for motionless wheels.
     */
    public SwerveModuleDeltas toSwerveModuleDelta(Twist2d twist) {
        // [dx; dy; dtheta] (3 x 1)
        SimpleMatrix twistVector = twist2Vector(twist);
        // [d cos; d sin; ...] (2n x 1)
        SimpleMatrix deltaVector = m_inverseKinematics.mult(twistVector);
        return deltasFromVector(deltaVector);
    }

    /**
     * Find the module deltas and apply them to the given initial positions.
     */
    public SwerveModulePositions toSwerveModulePositions(
            SwerveModulePositions initial,
            Twist2d twist) {
        SwerveModuleDeltas deltas = toSwerveModuleDelta(twist);
        return SwerveModulePositions.modulePositionFromDelta(initial, deltas);
    }

    /**
     * FORWARD: module states -> chassis speeds
     */
    public ChassisSpeeds toChassisSpeeds(SwerveModuleStates states) {
        // checkLength(states);
        // [v cos; v sin; ...] (2n x 1)
        SimpleMatrix statesVector = states2Vector(states);
        // [vx; vy; omega]
        SimpleMatrix chassisSpeedsVector = m_forwardKinematics.mult(statesVector);
        return vector2ChassisSpeeds(chassisSpeedsVector);
    }

    /**
     * FORWARD: module deltas -> twist.
     * 
     * assumes the module deltas represent straight lines.
     * 
     * NOTE: do not use the returned dtheta, use the gyro instead.
     * 
     * NOTE: this twist represents the "discrete" motion of the robot; don't use it
     * as if it were the instantaneous speed.
     */
    public Twist2d toTwist2d(SwerveModuleDeltas deltas) {
        // [d cos; d sin; ...] (2n x 1)
        SimpleMatrix deltaVector = deltas2Vector(deltas);
        // [dx ;dy; dtheta]
        SimpleMatrix twistVector = m_forwardKinematics.mult(deltaVector);
        return vector2Twist(twistVector);
    }

    ///////////////////////////////////////

    /** states -> [v cos; v sin; ... v cos; v sin] (2n x 1) */
    private SimpleMatrix states2Vector(SwerveModuleStates moduleStates) {
        SwerveModuleState100[] moduleStatesAll = moduleStates.all();
        SimpleMatrix moduleStatesMatrix = new SimpleMatrix(m_numModules * 2, 1);
        for (int i = 0; i < m_numModules; i++) {
            SwerveModuleState100 module = moduleStatesAll[i];
            if (Math.abs(module.speedMetersPerSecond()) < 1e-6 || module.angle().isEmpty()) {
                // wheel is stopped, or angle is invalid so pretend it's stopped.
                moduleStatesMatrix.set(i * 2, 0, 0);
                moduleStatesMatrix.set(i * 2 + 1, 0, 0);
            } else {
                moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond() * module.angle().get().getCos());
                moduleStatesMatrix.set(i * 2 + 1, 0, module.speedMetersPerSecond() * module.angle().get().getSin());

            }
        }
        return moduleStatesMatrix;
    }

    /**
     * produces a vector of corner dx and dy, assuming the module deltas represent
     * straight line paths.
     * 
     * @param moduleDeltas [d cos; d sin; ... ] (2n x 1)
     */
    private SimpleMatrix deltas2Vector(SwerveModuleDeltas moduleDeltas) {
        SwerveModuleDelta[] deltas = moduleDeltas.all();
        SimpleMatrix moduleDeltaMatrix = new SimpleMatrix(m_numModules * 2, 1);
        for (int i = 0; i < m_numModules; i++) {
            SwerveModuleDelta module = deltas[i];
            if (Math.abs(module.distanceMeters) < 1e-6 || module.wrappedAngle.isEmpty()) {
                moduleDeltaMatrix.set(i * 2, 0, 0);
                moduleDeltaMatrix.set(i * 2 + 1, 0, 0);
            } else {
                moduleDeltaMatrix.set(i * 2, 0, module.distanceMeters * module.wrappedAngle.get().getCos());
                moduleDeltaMatrix.set(i * 2 + 1, 0, module.distanceMeters * module.wrappedAngle.get().getSin());
            }
        }
        return moduleDeltaMatrix;
    }

    /** ChassisSpeeds -> [vx; vy; omega] (3 x 1) */
    private SimpleMatrix chassisSpeeds2Vector(DiscreteSpeed speed) {
        SimpleMatrix chassisSpeedsVector = new SimpleMatrix(3, 1);
        chassisSpeedsVector.setColumn(
                0,
                0,
                speed.twist().dx / speed.dt(),
                speed.twist().dy / speed.dt(),
                speed.twist().dtheta / speed.dt());
        return chassisSpeedsVector;
    }

    /** Twist as a 3x1 column vector: [dx; dy; dtheta] */
    private static SimpleMatrix twist2Vector(Twist2d twist) {
        return new SimpleMatrix(new double[] {
                twist.dx,
                twist.dy,
                twist.dtheta });
    }

    /** [vx; vy; omega] (3 x 1) -> ChassisSpeeds */
    private static ChassisSpeeds vector2ChassisSpeeds(SimpleMatrix v) {
        return new ChassisSpeeds(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    /** [dx; dy; dtheta] (3 x 1) -> Twist2d */
    private static Twist2d vector2Twist(SimpleMatrix v) {
        return new Twist2d(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    /**
     * [v cos; v sin; ... ] (2n x 1) -> states[]
     * 
     * Speed is always non-negative.
     * 
     * Angle can be empty if speed is about zero.
     * Otherwise angle is always within [-pi, pi].
     * 
     * @param chassisSpeedsVector [vx0; vy0; vx1; ...]
     */
    SwerveModuleStates statesFromVector(SimpleMatrix chassisSpeedsVector) {
        SimpleMatrix m = m_inverseKinematics.mult(chassisSpeedsVector);
        return new SwerveModuleStates(
                SwerveModuleState100.fromSpeed(m.get(0, 0), m.get(1, 0)),
                SwerveModuleState100.fromSpeed(m.get(2, 0), m.get(3, 0)),
                SwerveModuleState100.fromSpeed(m.get(4, 0), m.get(5, 0)),
                SwerveModuleState100.fromSpeed(m.get(6, 0), m.get(7, 0)));
    }

    public Translation2d[] getModuleLocations() {
        return m_moduleLocations;
    }

    /**
     * The resulting distance is always positive.
     * 
     * @param moduleDeltaVector [d cos; d sin; ...] (2n x 1),
     *                          equivalently [dx0; dy0; dx1; ...]
     */
    private SwerveModuleDeltas deltasFromVector(SimpleMatrix moduleDeltaVector) {
        return new SwerveModuleDeltas(
                new SwerveModuleDelta(moduleDeltaVector.get(0, 0), moduleDeltaVector.get(1, 0)),
                new SwerveModuleDelta(moduleDeltaVector.get(2, 0), moduleDeltaVector.get(3, 0)),
                new SwerveModuleDelta(moduleDeltaVector.get(4, 0), moduleDeltaVector.get(5, 0)),
                new SwerveModuleDelta(moduleDeltaVector.get(6, 0), moduleDeltaVector.get(7, 0)));
    }

    /** module locations -> inverse kinematics matrix (2n x 3) */
    private static SimpleMatrix inverseMatrix(Translation2d[] moduleLocations) {
        int numModules = moduleLocations.length;
        SimpleMatrix inverseKinematics = new SimpleMatrix(numModules * 2, 3);
        for (int i = 0; i < numModules; i++) {
            inverseKinematics.setRow(i * 2 + 0, 0, 1, 0, -moduleLocations[i].getY());
            inverseKinematics.setRow(i * 2 + 1, 0, 0, 1, +moduleLocations[i].getX());
        }
        return inverseKinematics;
    }

    private void checkModuleCount(Translation2d... moduleTranslationsM) {
        if (moduleTranslationsM.length < 2) {
            throw new IllegalArgumentException("Swerve requires at least two modules");
        }
    }
}
