package org.team100.lib.geometry;

public record GlobalJerkR3(double x, double y, double theta) {
    public double norm() {
        return Math.hypot(x, y);
    }

    public GlobalJerkR3 times(double scalar) {
        return new GlobalJerkR3(x * scalar, y * scalar, theta * scalar);
    }

    public GlobalAccelerationR3 integrate(double dtSec) {
        return new GlobalAccelerationR3(x * dtSec, y * dtSec, theta * dtSec);
    }
}
