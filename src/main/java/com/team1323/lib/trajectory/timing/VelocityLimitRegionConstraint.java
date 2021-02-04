package com.team1323.lib.trajectory.timing;

import com.team1323.lib.geometry.IUnwrappableTranslation2d;
import com.team1323.lib.geometry.UnwrappableTranslation2d;

public class VelocityLimitRegionConstraint<S extends IUnwrappableTranslation2d<S>> implements TimingConstraint<S> {
    protected final UnwrappableTranslation2d min_corner_;
    protected final UnwrappableTranslation2d max_corner_;
    protected final double velocity_limit_;

    public VelocityLimitRegionConstraint(UnwrappableTranslation2d min_corner, UnwrappableTranslation2d max_corner, double velocity_limit) {
        min_corner_ = min_corner;
        max_corner_ = max_corner;
        velocity_limit_ = velocity_limit;
    }

    @Override
    public double getMaxVelocity(S state) {
        final UnwrappableTranslation2d translation = state.getTranslation();
        if (translation.x() <= max_corner_.x() && translation.x() >= min_corner_.x() &&
                translation.y() <= max_corner_.y() && translation.y() >= min_corner_.y()) {
            return velocity_limit_;
        }
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public TimingConstraint.MinMaxAcceleration getMinMaxAcceleration(S state,
                                                                     double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }

}
