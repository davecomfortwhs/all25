package org.team100.lib.optimization;

import java.util.function.Function;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;

/**
 * Find the minimum of a multivariate scalar function.
 * 
 * @see https://en.wikipedia.org/wiki/Gradient_descent
 */
public class GradientDescent<R extends Num> {
    private final Nat<R> m_rows;
    private final Function<Vector<R>, Double> m_f;
    /** Return the current solution if a step doesn't improve it more than this. */
    private final double m_tolerance;
    private final int m_iterations;

    /**
     * Since we're actually optimizing the scalar error instead of an arbitrary
     * function, and we know the error is positive-definite, maybe this should just
     * say that.
     */
    public GradientDescent(
            Nat<R> rows,
            Function<Vector<R>, Double> f,
            double tolerance,
            int iterations) {
        m_rows = rows;
        m_f = f;
        m_tolerance = tolerance;
        m_iterations = iterations;
    }

    public Vector<R> solve(final Vector<R> initial) {
        double stepSize = 10;
        Vector<R> current = initial;
        double fCurrent = m_f.apply(current);
        for (int i = 0; i < m_iterations; ++i) {
            Vector<R> grad = NumericalGradient.numericalGradient(m_rows, m_f, current);
            Vector<R> gradDirection = grad.unit();
            Vector<R> next = current.plus(gradDirection.times(-1.0 * stepSize));
            double fNext = m_f.apply(next);
            // System.out.printf("current [%7.4f %7.4f] => %7.4f .. next [%7.4f %7.4f]
            // %7.4f\n",
            // current.get(0), current.get(1), fCurrent,
            // next.get(0), next.get(1), fNext);
            // System.out.printf("current %7.4f \n", fCurrent);
            // System.out.printf("%5.3f\n", current.get(0));
            if (fNext > fCurrent) {
                // if we go too far, turn down the step size and try again.
                stepSize *= 0.5;
                continue;
            }
            double step = Math.abs(fNext - fCurrent);
            if (step < m_tolerance) {
                return next;
            }
            current = next;
            fCurrent = fNext;
        }
        System.out.println("WARNING: did not meet tolerance");
        return current;
    }

}
