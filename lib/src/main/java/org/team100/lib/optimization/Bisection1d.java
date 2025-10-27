package org.team100.lib.optimization;

import java.util.function.DoubleUnaryOperator;

import org.team100.lib.util.Math100;

/**
 * Very simple scalar bisection solver.
 * 
 * @see https://en.wikipedia.org/wiki/Bisection_method
 */
public class Bisection1d {

    /**
     * @param func            to be solved
     * @param x_0             lower bound
     * @param f_0             f(x_0)
     * @param x_1             upper bound
     * @param f_1             f(x_1)
     * @param tolerance       how close to zero do we need
     * @param iterations_left iterations to go
     * @return s parameter
     */
    public static double findRoot(
            DoubleUnaryOperator func,
            double x_0,
            double f_0,
            double x_1,
            double f_1,
            double tolerance,
            int iterations_left) {
        if (Math100.DEBUG) {
            System.out.printf("*************** i %d x_0 %.8f f_0 %.8f x_1 %.8f f_1 %.8f\n",
                    iterations_left, x_0, f_0, x_1, f_1);
        }
        if (iterations_left < 0) {
            // ran out of time
            return 1.0;
        }
        if (Math.abs(f_0 - f_1) <= tolerance) {
            // failed to find a solution
            return 1.0;
        }
        if (Math.abs(f_0) < tolerance) {
            if (Math100.DEBUG)
                System.out.println("left edge is the solution");
            return x_0;
        }
        if (Math.abs(f_1) < tolerance) {
            if (Math100.DEBUG)
                System.out.println("right edge is the solution");
            return x_1;
        }
        // halfway between x_0 and x_1.
        final double s_guess = 0.5;

        double x_guess = (x_1 - x_0) * s_guess + x_0;
        double f_guess = func.applyAsDouble(x_guess);
        if (Math100.DEBUG) {
            System.out.printf("************* guess f(%.8f) = %.8f\n", x_guess, f_guess);
        }

        if (Math.abs(f_guess) < tolerance) {
            if (Math100.DEBUG) {
                System.out.printf("guess %.8f less than tolerance %.8f\n", f_guess, tolerance);
            }
            return s_guess;
        }

        if (Math.signum(f_0) == Math.signum(f_guess)) {
            // 0 and guess on same side of root, so use upper bracket.
            return s_guess
                    + (1.0 - s_guess) * findRoot(func, x_guess, f_guess, x_1, f_1, tolerance, iterations_left - 1);
        } else {
            // Use lower bracket.
            return s_guess * findRoot(func, x_0, f_0, x_guess, f_guess, tolerance, iterations_left - 1);
        }
    }

}
