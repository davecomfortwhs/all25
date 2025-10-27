package org.team100.lib.motion.prr;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record JointAccelerations(double elevator, double shoulder, double wrist) {
    public Vector<N3> toVector() {
        return VecBuilder.fill(elevator, shoulder, wrist);
    }

    public static JointAccelerations fromVector(Matrix<N3, N1> a) {
        return new JointAccelerations(a.get(0, 0), a.get(1, 0), a.get(2, 0));
    }

    public Vector<N3> div(JointAccelerations ja) {
        return VecBuilder.fill(
                elevator / ja.elevator,
                shoulder / ja.shoulder,
                wrist / ja.wrist);
    }

    public double norm() {
        return Math.sqrt(elevator * elevator + shoulder * shoulder + wrist * wrist);
    }

    public JointAccelerations times(double s) {
        return new JointAccelerations(s * elevator, s * shoulder, s * wrist);
    }
}
