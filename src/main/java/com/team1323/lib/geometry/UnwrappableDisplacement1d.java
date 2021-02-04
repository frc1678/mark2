package com.team1323.lib.geometry;

import com.team254.lib.geometry.State;
import com.team254.lib.util.Util;

import java.text.DecimalFormat;

public class UnwrappableDisplacement1d implements State<UnwrappableDisplacement1d> {

    protected final double displacement_;

    public UnwrappableDisplacement1d() {
        displacement_ = 0.0;
    }

    public UnwrappableDisplacement1d(double displacement) {
        displacement_ = displacement;
    }

    public double x() {
        return displacement_;
    }

    @Override
    public UnwrappableDisplacement1d interpolate(final UnwrappableDisplacement1d other, double x) {
        return new UnwrappableDisplacement1d(Util.interpolate(displacement_, other.displacement_, x));
    }

    @Override
    public double distance(final UnwrappableDisplacement1d other) {
        return Math.abs(x() - other.x());
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof UnwrappableDisplacement1d))
            return false;
        return Util.epsilonEquals(x(), ((UnwrappableDisplacement1d) other).x());
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format("(" + x() + ")");
    }

    @Override
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(x());
    }
}
