package org.team100.lib.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;

public class StrUtil {

    public static String pose2Str(Pose2d p) {
        return String.format("%6.3f, %6.3f, %6.3f",
                p.getX(), p.getY(), p.getRotation().getRadians());
    }

    public static String poseStr(Pose3d p) {
        return String.format("%.8e, %.8e, %.8e, %.8e, %.8e, %.8e",
                p.getX(), p.getY(), p.getZ(),
                p.getRotation().getX(), p.getRotation().getY(), p.getRotation().getZ());
    }

    public static String twistStr(Twist3d t) {
        return String.format("Twist3d: [%.8e %.8e %.8e %.8e %.8e %.8e]",
                t.dx, t.dy, t.dz, t.rx, t.ry, t.rz);
    }

    public static String rotStr(Rotation3d r) {
        return String.format("%.8e, %.8e, %.8e",
                r.getX(), r.getY(), r.getZ());
    }

    public static String transStr(Translation3d r) {
        return String.format("%.8e, %.8e, %.8e",
                r.getX(), r.getY(), r.getZ());
    }

    public static String vecStr(Vector<?> m) {
        StringBuilder b = new StringBuilder();
        b.append("[");
        for (int i = 0; i < m.getNumRows(); ++i) {
            b.append(String.format(" %.10e", m.get(i)));
        }
        b.append("]");
        return b.toString();
    }

    public static String matStr(Matrix<?, ?> m) {
        StringBuilder b = new StringBuilder();
        b.append("[");
        for (int i = 0; i < m.getNumRows(); ++i) {
            for (int j = 0; j < m.getNumCols(); ++j) {
                b.append(String.format(" %.3e", m.get(i, j)));
            }
            b.append(";\n");
        }

        b.append("]");
        return b.toString();
    }

    private StrUtil() {
        //
    }
}
