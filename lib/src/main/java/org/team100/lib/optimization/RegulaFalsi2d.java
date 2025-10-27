package org.team100.lib.optimization;

import java.util.function.DoubleBinaryOperator;

/**
 * Two-dimensional regula falsi solver.
 * 
 * This was used by 254 in 2023 to find optimal drive wheel speeds, which isn't
 * something we do anymore, so ...
 * 
 * @see https://en.wikipedia.org/wiki/Regula_falsi
 */
public class RegulaFalsi2d {
    private static final double ROOT_TOLERANCE = 0.0001;

    /**
     * Find the root of the generic 2D parametric function 'func' using the regula
     * falsi technique. This is a pretty naive way to do root finding, but it's
     * usually faster than simple bisection while being robust in ways that e.g. the
     * Newton-Raphson method isn't.
     * 
     * @param func            The Function2d to take the root of.
     * @param x_0             x value of the lower bracket.
     * @param y_0             y value of the lower bracket.
     * @param f_0             value of 'func' at x_0, y_0 (passed in by caller to
     *                        save a call to 'func' during recursion)
     * @param x_1             x value of the upper bracket.
     * @param y_1             y value of the upper bracket.
     * @param f_1             value of 'func' at x_1, y_1 (passed in by caller to
     *                        save a call to 'func' during recursion)
     * @param iterations_left Number of iterations of root finding left.
     * @return The parameter value 's' that interpolating between 0 and 1 that
     *         corresponds to the (approximate) root.
     */
    public static double findRoot(
            DoubleBinaryOperator func,
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            int iterations_left) {

        if (iterations_left < 0) {
            return 1.0;
        }
        if (Math.abs(f_0 - f_1) <= ROOT_TOLERANCE) {
            return 1.0;
        }
        // interpolation parameter
        double s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));
        double x_guess = (x_1 - x_0) * s_guess + x_0;
        double y_guess = (y_1 - y_0) * s_guess + y_0;
        double f_guess = func.applyAsDouble(x_guess, y_guess);

        if (Math.abs(f_guess) < ROOT_TOLERANCE) {
            // this is new as of dec 2023, why wasn't this here before?
            return s_guess;
        }

        if (Math.signum(f_0) == Math.signum(f_guess)) {
            // 0 and guess on same side of root, so use upper bracket.
            return s_guess
                    + (1.0 - s_guess) * findRoot(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
        } else {
            // Use lower bracket.
            return s_guess * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
        }
    }
}
