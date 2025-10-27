package org.team100.lib.optimization;

import java.util.Random;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.jni.EigenJNI;

/**
 * Newton's method finds a zero of a multivariate function.
 * 
 * For example, supply a function that describes the error in estimate and goal:
 * driving it to zero yields the x values to get the desired f.
 * 
 * Uses the (estimated) Jacobian of the function to estimate the x intercept.
 * 
 * https://en.wikipedia.org/wiki/Newton%27s_method
 * https://hades.mech.northwestern.edu/images/7/7f/MR.pdf
 */
public class NewtonsMethod<X extends Num, Y extends Num> {
    private static final boolean DEBUG = false;
    private final Nat<X> m_xdim;
    private final Nat<Y> m_ydim;
    private final Function<Vector<X>, Vector<Y>> m_f;
    private final Vector<X> m_xMin;
    private final Vector<X> m_xMax;
    private final double m_tolerance;
    private final int m_iterations;
    /**
     * Max change in estimate per iteration, to avoid overreacting.
     */
    private final double m_dxLimit;

    private final Random random = new Random();

    /**
     * 
     * @param xdim
     * @param ydim
     * @param f          The f function should take euclidean (i.e. independent)
     *                   input and return an error estimate in the tangent manifold
     *                   of whatever space it's actually acting in (e.g. SE(3))
     * @param xMin
     * @param xMax
     * @param tolerance
     * @param iterations when Newton gets stuck, it only takes a few iterations, so
     *                   this doesn't need to be large. Specify a generous
     *                   random-restart limit instead of a large iteration limit.
     * @param dxLimit
     */
    public NewtonsMethod(
            Nat<X> xdim,
            Nat<Y> ydim,
            Function<Vector<X>, Vector<Y>> f,
            Vector<X> xMin,
            Vector<X> xMax,
            double tolerance,
            int iterations,
            double dxLimit) {
        m_xdim = xdim;
        m_ydim = ydim;
        m_f = f;
        m_xMin = xMin;
        m_xMax = xMax;
        m_tolerance = tolerance;
        m_iterations = iterations;
        m_dxLimit = dxLimit;
    }

    /** Symmetric Jacobian, slower. */
    public Vector<X> solve(Vector<X> initial) {
        Vector<X> x = new Vector<>(initial.getStorage().copy());
        for (int i = 0; i < m_iterations; ++i) {
            Vector<Y> error = m_f.apply(x);
            if (within(error)) {
                return x;
            }
            Matrix<Y, X> j = NumericalJacobian100.numericalJacobian(m_xdim, m_ydim, m_f, x);
            Vector<X> dx = new Vector<>(j.solve(error));
            // Too-high dx results in oscillation.
            clamp(dx);
            update(x, dx);
            // Keep the x estimate within bounds.
            limit(x);
        }
        return x;
    }

    /**
     * Single-sided Jacobian, faster.
     * 
     * @param initialX start here
     * @param restarts number of random restarts in case of non-convergence
     */
    public Vector<X> solve2(Vector<X> initialX, int restarts) {
        // System.out.printf("initialX: %s\n", StrUtil.vecStr(initialX));
        long startTime = System.nanoTime();
        int iter = 0;
        Vector<Y> error = new Vector<>(m_ydim);
        try {
            // x is the solution estimate
            Vector<X> x = new Vector<>(initialX.getStorage().copy());
            for (iter = 0; iter < m_iterations; ++iter) {
                // if (DEBUG)
                // System.out.printf("x: %s\n", StrUtil.vecStr(x));

                error = m_f.apply(x);
                // if (DEBUG)
                // System.out.printf("error: %s\n", StrUtil.vecStr(error));

                if (within(error)) {
                    // System.out.println("success");
                    return x;
                }
                Matrix<Y, X> j = NumericalJacobian100.numericalJacobian2(m_xdim, m_ydim, m_f, x);

                // if (DEBUG)
                // System.out.printf("J %s\n", StrUtil.matStr(j));

                // the pseudoinverse should always work too
                // note this is quite slow
                // Matrix<X, Y> jInv = new Matrix<>(j.getStorage().pseudoInverse());
                // if (DEBUG)
                // System.out.printf("Jinv %s\n", StrUtil.matStr(jInv));

                // Vector<X> dx = new Vector<>(jInv.times(error));

                // this method is faster.
                // solve A x = B i.e. J dx = error
                Vector<X> dx = new Vector<>(j.solve(error));

                // this solver also works but it's not better.
                // Vector<X> dx = getDxWithQRDecomp(error, j);

                // if (DEBUG)
                // System.out.printf("dx: %s\n", StrUtil.vecStr(dx));

                // Too-high dx results in oscillation.
                clamp(dx);
                update(x, dx);
                // Keep the x estimate within bounds.
                limit(x);
            }
            if (restarts > 0) {
                // if (DEBUG)
                System.out.println("convergence failed, trying random restart");
                // this is *really* random, including out-of-bounds.
                // SimpleMatrix r = SimpleMatrix.random(m_xdim.getNum(), 1)
                // .minus(0.5)
                // .scale(1);
                // Vector<X> rv = new Vector<>(r);
                // System.out.printf("rv %s\n", StrUtil.vecStr(rv));
                // rv = rv.plus(x);
                // limit(rv);
                for (int i = 0; i < m_xdim.getNum(); i++) {
                    x.set(i, 0, x.get(i) + 0.1 * (random.nextDouble() - 0.5));
                }
                limit(x);
                return solve2(x, restarts - 1);
            }
            // if (DEBUG)
            System.out.printf("random restart failed, error %f\n", error.maxAbs());
            // throw new IllegalArgumentException(
            // String.format("failed to converge for inputs %s",
            // StrUtil.vecStr(initialX)));
            return x;
        } finally {
            long finishTime = System.nanoTime();
            if (DEBUG) {
                System.out.printf("solve2 iterations: %d ET (ms): %6.3f\n", iter,
                        ((double) finishTime - startTime) / 1000000);
            }
        }
    }

    /** A different solver */
    Vector<X> getDxWithQRDecomp(Vector<Y> error, Matrix<Y, X> j) {
        double[] A = j.getData();
        int Arows = j.getNumRows();
        int Acols = j.getNumCols();
        double[] B = error.getData();
        int Brows = error.getNumRows();
        int Bcols = error.getNumCols();
        // dst has same dimensions as B
        double[] dst = new double[B.length];
        // solve Ax=B
        EigenJNI.solveFullPivHouseholderQr(A, Arows, Acols, B, Brows, Bcols, dst);
        Vector<X> dx = new Vector<>(new Matrix<>(m_xdim, Nat.N1(), dst));
        return dx;
    }

    private boolean within(Vector<Y> error) {
        return error.maxAbs() < m_tolerance;
    }

    /**
     * Clamp the guess vector x to the per-dimension limits.
     * Mutates x to save allocations.
     */
    private void limit(Vector<X> x) {
        for (int i = 0; i < x.getNumRows(); ++i) {
            double xi = x.get(i);
            double xMin = m_xMin.get(i);
            double xMax = m_xMax.get(i);
            xi = MathUtil.clamp(xi, xMin, xMax);
            x.set(i, 0, xi);
        }
    }

    /**
     * Update: x = x + dx.
     * Mutates x to save allocations.
     * The "x" space is Euclidean, so using a simple sum is ok.
     */
    private void update(Vector<X> x, Vector<X> dx) {
        for (int i = 0; i < x.getNumRows(); ++i) {
            double newXi = x.get(i) - dx.get(i);
            x.set(i, 0, newXi);
        }
    }

    /**
     * Limit: dx = clamp(dx) using a fixed limit.
     * Mutates dx to save allocations.
     */
    private void clamp(Vector<X> dx) {
        for (int i = 0; i < dx.getNumRows(); ++i) {
            double dxI = dx.get(i);
            if (Math.abs(dxI) > m_dxLimit) {
                if (DEBUG)
                    System.out.println("clamped!");
            }
            double clampedDxI = MathUtil.clamp(dxI, -m_dxLimit, m_dxLimit);
            // System.out.printf("clamp %d %15.10f %15.10f\n", i, dxI, clampedDxI);
            dx.set(i, 0, clampedDxI);
        }
    }
}
