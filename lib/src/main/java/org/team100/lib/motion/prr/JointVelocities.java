package org.team100.lib.motion.prr;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * These are PRR velocities.
 * 
 * Use the jacobian to find these from a cartesian velocity.
 */
public record JointVelocities(double elevator, double shoulder, double wrist) {
    public static JointVelocities fromVector(Vector<N3> v) {
        return new JointVelocities(v.get(0), v.get(1), v.get(2));
    }

    public static JointVelocities fromVector(Matrix<N3, N1> v) {
        return new JointVelocities(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(elevator, shoulder, wrist);
    }

    public JointAccelerations diff(JointVelocities jv, double dt) {
        return new JointAccelerations(
                (elevator - jv.elevator) / dt,
                (shoulder - jv.shoulder) / dt,
                (wrist - jv.wrist) / dt);
    }

    public Vector<N3> div(JointVelocities jv) {
        return VecBuilder.fill(
                elevator / jv.elevator,
                shoulder / jv.shoulder,
                wrist / jv.wrist);
    }

    public double norm() {
        return Math.sqrt(elevator * elevator + shoulder * shoulder + wrist * wrist);
    }

    public JointVelocities times(double s) {
        return new JointVelocities(s * elevator, s * shoulder, s * wrist);
    }
}
