package com.team1323.lib.geometry;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.Util;

import java.text.DecimalFormat;

import static com.team254.lib.util.Util.kEpsilon;

/**
 * A rotation in a 2d coordinate frame represented a point on the unit circle (cosine and sine).
 * <p>
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
public class UnwrappableRotation2d implements IUnwrappableRotation2d<UnwrappableRotation2d> {
    protected static final UnwrappableRotation2d kIdentity = new UnwrappableRotation2d();

    public static final UnwrappableRotation2d identity() {
        return kIdentity;
    }

    protected final double cos_angle_;
    protected final double sin_angle_;
    protected double theta_degrees = 0;
    protected double theta_radians = 0;

    public UnwrappableRotation2d() {
        this(1, 0, false);
    }

    public UnwrappableRotation2d(double x, double y, boolean normalize) {
        if (normalize) {
            // From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object
            // we might accumulate rounding errors.
            // Normalizing forces us to re-scale the sin and cos to reset rounding errors.
            double magnitude = Math.hypot(x, y);
            if (magnitude > kEpsilon) {
                sin_angle_ = y / magnitude;
                cos_angle_ = x / magnitude;
            } else {
                sin_angle_ = 0;
                cos_angle_ = 1;
            }
        } else {
            cos_angle_ = x;
            sin_angle_ = y;
        }
        theta_degrees = Math.toDegrees(Math.atan2(sin_angle_, cos_angle_));
    }

    public UnwrappableRotation2d(final UnwrappableRotation2d other) {
        cos_angle_ = other.cos_angle_;
        sin_angle_ = other.sin_angle_;
        theta_degrees = Math.toDegrees(Math.atan2(sin_angle_, cos_angle_));
    }

    public UnwrappableRotation2d(double theta_degrees) {
        cos_angle_ = Math.cos(Math.toRadians(theta_degrees));
        sin_angle_ = Math.sin(Math.toRadians(theta_degrees));
        this.theta_degrees = theta_degrees;
    }

    public UnwrappableRotation2d(final UnwrappableTranslation2d direction, boolean normalize) {
        this(direction.x(), direction.y(), normalize);
    }

    public Rotation2d wrap() {
        return Rotation2d.fromDegrees(this.theta_degrees);
    }

    public static UnwrappableRotation2d fromRadians(double angle_radians) {
        return new UnwrappableRotation2d(Math.cos(angle_radians), Math.sin(angle_radians), false);
    }

    public static UnwrappableRotation2d fromDegrees(double angle_degrees) {
        return new UnwrappableRotation2d(angle_degrees);
    }

    public double cos() {
        return cos_angle_;
    }

    public double sin() {
        return sin_angle_;
    }

    public double tan() {
        if (Math.abs(cos_angle_) < kEpsilon) {
            if (sin_angle_ >= 0.0) {
                return Double.POSITIVE_INFINITY;
            } else {
                return Double.NEGATIVE_INFINITY;
            }
        }
        return sin_angle_ / cos_angle_;
    }

    public double getRadians() {
        return Math.atan2(sin_angle_, cos_angle_);
    }

    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }

    public double getUnboundedDegrees() {
        return theta_degrees;
    }

    /**
     * We can rotate this Rotation2d by adding together the effects of it and
     * another rotation.
     *
     * @param other The other rotation. See:
     *              https://en.wikipedia.org/wiki/Rotation_matrix
     * @return This rotation rotated by other.
     */
    public UnwrappableRotation2d rotateBy(final UnwrappableRotation2d other) {
        return new UnwrappableRotation2d(cos_angle_ * other.cos_angle_ - sin_angle_ * other.sin_angle_,
                cos_angle_ * other.sin_angle_ + sin_angle_ * other.cos_angle_, true);
    }

    public UnwrappableRotation2d normal() {
        return new UnwrappableRotation2d(-sin_angle_, cos_angle_, false);
    }

    /**
     * Multiplies the current rotation by a scalar.
     *
     * @param scalar The scalar.
     * @return The new scaled Rotation2d.
     */
    public UnwrappableRotation2d times(double scalar) {
        return UnwrappableRotation2d.fromRadians(getRadians() * scalar);
    }

    /**
     * The inverse of a Rotation2d "undoes" the effect of this rotation.
     *
     * @return The opposite of this rotation.
     */
    public UnwrappableRotation2d inverse() {
        return new UnwrappableRotation2d(cos_angle_, -sin_angle_, false);
    }

    public boolean isParallel(final UnwrappableRotation2d other) {
        return Util.epsilonEquals(UnwrappableTranslation2d.cross(toTranslation(), other.toTranslation()), 0.0);
    }

    public UnwrappableTranslation2d toTranslation() {
        return new UnwrappableTranslation2d(cos_angle_, sin_angle_);
    }

    public UnwrappableRotation2d toNegative() {
        return UnwrappableRotation2d.fromDegrees(-theta_degrees);
    }

    /**
     * @return The pole nearest to this rotation.
     */
    public UnwrappableRotation2d nearestPole() {
        double pole_sin = 0.0;
        double pole_cos = 0.0;
        if (Math.abs(cos_angle_) > Math.abs(sin_angle_)) {
            pole_cos = Math.signum(cos_angle_);
            pole_sin = 0.0;
        } else {
            pole_cos = 0.0;
            pole_sin = Math.signum(sin_angle_);
        }
        return new UnwrappableRotation2d(pole_cos, pole_sin, false);
    }

    @Override
    public UnwrappableRotation2d interpolate(final UnwrappableRotation2d other, double x) {
        if (x <= 0) {
            return new UnwrappableRotation2d(this);
        } else if (x >= 1) {
            return new UnwrappableRotation2d(other);
        }
        double angle_diff = inverse().rotateBy(other).getRadians();
        return this.rotateBy(UnwrappableRotation2d.fromRadians(angle_diff * x));
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(getDegrees()) + " deg)";
    }

    @Override
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(getDegrees());
    }

    @Override
    public double distance(final UnwrappableRotation2d other) {
        return inverse().rotateBy(other).getRadians();
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof UnwrappableRotation2d))
            return false;
        return distance((UnwrappableRotation2d) other) < Util.kEpsilon;
    }

    @Override
    public UnwrappableRotation2d getRotation() {
        return this;
    }
}
