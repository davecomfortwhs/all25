package org.team100.lib.optimization;

import java.util.function.DoubleUnaryOperator;

/**
 * Golden ratio search finds the minimum by testing points. If the function has
 * more than one local minimum, there's no guarantee it will find the global
 * minimum.
 * 
 * It's intended to find the minimum error in each step of coordinate descent,
 * so the function has a scalar input (i.e. configuration of one joint) and a
 * scalar output (i.e. distance).
 * 
 * This uses the optimal partitioning and only evaluates once per step.
 * 
 * This is faster than the TernarySearch method mainly because it only evaluates
 * once per step.
 * 
 * @see https://en.wikipedia.org/wiki/Golden-section_search
 */
public class GoldenSectionSearch {
    private static final double INVPHI = (Math.sqrt(5) - 1) / 2; // 1 / phi

    private final DoubleUnaryOperator m_f;
    private final double m_tolerance;
    private final int m_iterations;

    public GoldenSectionSearch(DoubleUnaryOperator f, double tolerance, int iterations) {
        m_f = f;
        m_tolerance = tolerance;
        m_iterations = iterations;
    }

    /** Find the minimum of f within [a,b]. */
    public double solve(double bottom, double top) {
        if (bottom > top)
            throw new IllegalArgumentException("bottom must be less than top");
        int i = 0;
        // Initial evaluations
        final double d = (top - bottom) * INVPHI;
        // NOTE: d is more than half way
        double low = top - d;
        double high = bottom + d;
        double flow = m_f.applyAsDouble(low);
        double fhigh = m_f.applyAsDouble(high);

        while (top - bottom > m_tolerance) {
            // System.out.printf("%5.3f %5.3f\n", bottom, top);
            if (flow < fhigh) {
                top = high;
                high = low;
                fhigh = flow;
                low = top - (top - bottom) * INVPHI;
                flow = m_f.applyAsDouble(low);
            } else {
                bottom = low;
                low = high;
                flow = fhigh;
                high = bottom + (top - bottom) * INVPHI;
                fhigh = m_f.applyAsDouble(high);
            }
            i++;
            if (i > m_iterations) {
                System.out.println("WARNING: iteration limit exceeded");
                return bottom;
            }
        }
        return bottom;
    }

}