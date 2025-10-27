package org.team100.lib.optimization;

import java.util.function.Function;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;

/**
 * Similar to the WPI version but using vectors instead of matrices.
 * 
 * Estimates the Jacobian using symmetric differences around the reference x.
 */
public class NumericalJacobian100 {
    private static final double DX = 1e-5;

    /**
     * Estimates the Jacobian using symmetric differences around the reference x.
     */
    public static <Y extends Num, X extends Num> Matrix<Y, X> numericalJacobian(
            Nat<X> xdim,
            Nat<Y> ydim,
            Function<Vector<X>, Vector<Y>> f,
            Vector<X> x) {
        Matrix<Y, X> result = new Matrix<>(ydim, xdim);
        for (int i = 0; i < xdim.getNum(); i++) {
            Vector<X> dxPlus = new Vector<>(x.getStorage().copy());
            Vector<X> dxMinus = new Vector<>(x.getStorage().copy());
            dxPlus.set(i, 0, dxPlus.get(i, 0) + DX);
            dxMinus.set(i, 0, dxMinus.get(i, 0) - DX);
            Vector<Y> dF = f.apply(dxPlus).minus(f.apply(dxMinus)).div(2 * DX);
            result.setColumn(i, Matrix.changeBoundsUnchecked(dF));
        }
        return result;
    }

    /**
     * Estimates the Jacobian using a single-sided difference to the right of the
     * reference x.
     * 
     * Mutates x to save allocation cost, but puts it back the way it was upon
     * returning.
     */
    public static <Y extends Num, X extends Num> Matrix<Y, X> numericalJacobian2(
            Nat<X> xdim,
            Nat<Y> ydim,
            Function<Vector<X>, Vector<Y>> f,
            Vector<X> x) {
        final Vector<Y> Y = f.apply(x);
        // System.out.printf("*** x %s Y %s\n", StrUtil.vecStr(x), StrUtil.vecStr(Y));
        Matrix<Y, X> result = new Matrix<>(ydim, xdim);
        for (int colI = 0; colI < xdim.getNum(); colI++) {
            final double xi = x.get(colI);
            x.set(colI, 0, xi + DX);
            final Vector<Y> Y1 = f.apply(x);
            // System.out.printf("x %s Y1 %s\n", StrUtil.vecStr(x),  StrUtil.vecStr(Y1));
            for (int rowI = 0; rowI < ydim.getNum(); rowI++) {
                double dy = Y1.get(rowI) - Y.get(rowI);
                double dydx = dy / DX;
                result.set(rowI, colI, dydx);
            }
            // put the old value back
            x.set(colI, 0, xi);
        }
        return result;
    }
}
