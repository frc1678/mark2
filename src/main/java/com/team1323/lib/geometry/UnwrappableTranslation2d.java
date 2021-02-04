package com.team1323.lib.geometry;

import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;

import java.text.DecimalFormat;

/**
 * A translation in a 2d coordinate frame. Translations are simply shifts in an (x, y) plane.
 */
public class UnwrappableTranslation2d implements IUnwrappableTranslation2d<UnwrappableTranslation2d> {
    protected static final UnwrappableTranslation2d kIdentity = new UnwrappableTranslation2d();

    public static final UnwrappableTranslation2d identity() {
        return kIdentity;
    }

    protected double x_;
    protected double y_;

    public UnwrappableTranslation2d() {
        x_ = 0;
        y_ = 0;
    }

    public UnwrappableTranslation2d(double x, double y) {
        x_ = x;
        y_ = y;
    }

    public UnwrappableTranslation2d(final UnwrappableTranslation2d other) {
        x_ = other.x_;
        y_ = other.y_;
    }

    public UnwrappableTranslation2d(final UnwrappableTranslation2d start, final UnwrappableTranslation2d end) {
        x_ = end.x_ - start.x_;
        y_ = end.y_ - start.y_;
    }

    public static UnwrappableTranslation2d fromPolar(UnwrappableRotation2d direction, double magnitude) {
        return new UnwrappableTranslation2d(direction.cos() * magnitude, direction.sin() * magnitude);
    }

    public Translation2d wrap() {
        return new Translation2d(x_, y_);
    }

    /**
     * The "norm" of a transform is the Euclidean distance in x and y.
     *
     * @return sqrt(x ^ 2 + y ^ 2)
     */
    public double norm() {
        return Math.hypot(x_, y_);
    }

    public double norm2() {
        return x_ * x_ + y_ * y_;
    }

    /**
     * Normalizing a vector scales it so that its norm is 1 while maintaining its
     * direction. If input is a zero vector, return a zero vector.
     * 
     * @return r / norm(r) or (0,0)
     */
    public UnwrappableTranslation2d normalize() {
        if (epsilonEquals(identity(), Util.kEpsilon))
            return this;
        return scale(1.0 / norm());
    }

    public double x() {
        return x_;
    }

    public double y() {
        return y_;
    }

    public void setX(double x) {
        x_ = x;
    }

    public void setY(double y) {
        y_ = y;
    }

    /**
     * We can compose Translation2d's by adding together the x and y shifts.
     *
     * @param other The other translation to add.
     * @return The combined effect of translating by this object and the other.
     */
    public UnwrappableTranslation2d translateBy(final UnwrappableTranslation2d other) {
        return new UnwrappableTranslation2d(x_ + other.x_, y_ + other.y_);
    }

    /**
     * We can also rotate Translation2d's. See:
     * https://en.wikipedia.org/wiki/Rotation_matrix
     *
     * @param rotation The rotation to apply.
     * @return This translation rotated by rotation.
     */
    public UnwrappableTranslation2d rotateBy(final UnwrappableRotation2d rotation) {
        return new UnwrappableTranslation2d(x_ * rotation.cos() - y_ * rotation.sin(),
                x_ * rotation.sin() + y_ * rotation.cos());
    }

    public UnwrappableRotation2d direction() {
        return new UnwrappableRotation2d(x_, y_, true);
    }

    /**
     * The inverse simply means a Translation2d that "undoes" this object.
     *
     * @return Translation by -x and -y.
     */
    public UnwrappableTranslation2d inverse() {
        return new UnwrappableTranslation2d(-x_, -y_);
    }

    @Override
    public UnwrappableTranslation2d interpolate(final UnwrappableTranslation2d other, double x) {
        if (x <= 0) {
            return new UnwrappableTranslation2d(this);
        } else if (x >= 1) {
            return new UnwrappableTranslation2d(other);
        }
        return extrapolate(other, x);
    }

    public UnwrappableTranslation2d extrapolate(final UnwrappableTranslation2d other, double x) {
        return new UnwrappableTranslation2d(x * (other.x_ - x_) + x_, x * (other.y_ - y_) + y_);
    }

    public UnwrappableTranslation2d scale(double s) {
        return new UnwrappableTranslation2d(x_ * s, y_ * s);
    }

    public boolean epsilonEquals(final UnwrappableTranslation2d other, double epsilon) {
        return Util.epsilonEquals(x(), other.x(), epsilon) && Util.epsilonEquals(y(), other.y(), epsilon);
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(x_) + "," + fmt.format(y_) + ")";
    }

    @Override
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(x_) + "," + fmt.format(y_);
    }

    public static double dot(final UnwrappableTranslation2d a, final UnwrappableTranslation2d b) {
        return a.x_ * b.x_ + a.y_ * b.y_;
    }

    /**
     * The scalar projection of a vector u onto a vector v is the length of the
     * "shadow" cast by u onto v under a "light" that is placed on a line normal to
     * v and containing the endpoint of u, given that u and v share a starting
     * point. tl;dr: _* u /| / | / | v *---+---------------->* \___/ | scal_v(u)
     * u.scal(v)
     * 
     * @return (u . v) / norm(v)
     */
    public double scal(UnwrappableTranslation2d v) {
        return dot(this, v) / v.norm();
    }

    /**
     * The projection of a vector u onto a vector v is the vector in the direction
     * of v with the magnitude u.scal(v).
     * 
     * @return u.scal(v) * v / norm(v)
     */
    public UnwrappableTranslation2d proj(UnwrappableTranslation2d v) {
        return v.normalize().scale(scal(v));
    }

    /**
     * https://stackoverflow.com/a/1167047/6627273 A point D is considered "within"
     * an angle ABC when cos(DBM) > cos(ABM) where M is the midpoint of AC, so ABM
     * is half the angle ABC. The cosine of an angle can be computed as the dot
     * product of two normalized vectors in the directions of its sides. Note that
     * this definition of "within" does not include points that lie on the sides of
     * the given angle. If `vertical` is true, then check not within the given
     * angle, but within the image of that angle rotated by pi about its vertex.
     * 
     * @param Translation2d A A point on one side of the angle.
     * @param Translation2d B The vertex of the angle.
     * @param Translation2d C A point on the other side of the angle.
     * @param boolean       vertical Whether to check in the angle vertical to the
     *                      one given
     * @return Whether this translation is within the given angle.
     * @author Joseph Reed
     */
    public boolean isWithinAngle(UnwrappableTranslation2d A, UnwrappableTranslation2d B, UnwrappableTranslation2d C,
            boolean vertical) {
        UnwrappableTranslation2d M = A.interpolate(C, 0.5); // midpoint
        UnwrappableTranslation2d m = (new UnwrappableTranslation2d(B, M)).normalize(); // mid-vector
        UnwrappableTranslation2d a = (new UnwrappableTranslation2d(B, A)).normalize(); // side vector
        UnwrappableTranslation2d d = (new UnwrappableTranslation2d(B, this)).normalize(); // vector to here
        if (vertical) {
            m = m.inverse();
            a = a.inverse();
        }
        return UnwrappableTranslation2d.dot(d, m) > UnwrappableTranslation2d.dot(a, m);
    }

    public boolean isWithinAngle(UnwrappableTranslation2d A, UnwrappableTranslation2d B, UnwrappableTranslation2d C) {
        return isWithinAngle(A, B, C, false);
    }

    /** Assumes an angle centered at the origin. */
    public boolean isWithinAngle(UnwrappableTranslation2d A, UnwrappableTranslation2d C, boolean vertical) {
        return isWithinAngle(A, identity(), C, vertical);
    }

    public boolean isWithinAngle(UnwrappableTranslation2d A, UnwrappableTranslation2d C) {
        return isWithinAngle(A, C, false);
    }

    public static UnwrappableRotation2d getAngle(final UnwrappableTranslation2d a, final UnwrappableTranslation2d b) {
        double cos_angle = dot(a, b) / (a.norm() * b.norm());
        if (Double.isNaN(cos_angle)) {
            return new UnwrappableRotation2d();
        }
        return UnwrappableRotation2d.fromRadians(Math.acos(Math.min(1.0, Math.max(cos_angle, -1.0))));
    }

    public static double cross(final UnwrappableTranslation2d a, final UnwrappableTranslation2d b) {
        return a.x_ * b.y_ - a.y_ * b.x_;
    }

    /**
     * The distance between a point and a line can be computed as a scalar
     * projection.
     * 
     * @param Translation2d a One point on the line.
     * @param Translation2d b Another point on the line.
     */
    public double distanceToLine(UnwrappableTranslation2d a, UnwrappableTranslation2d b) {
        UnwrappableTranslation2d point = new UnwrappableTranslation2d(a, this);
        UnwrappableTranslation2d line = new UnwrappableTranslation2d(a, b);
        UnwrappableTranslation2d perpLine = line.rotateBy(new UnwrappableRotation2d(90));
        return Math.abs(point.scal(perpLine));
        // let's use readable code from now on, not golfed one-liners, shall we?
        // return Math.abs((new Translation2d(a,this))scal((new
        // Translation2d(a,b)).rotateBy(new Rotation2d(90))));
    }

    @Override
    public double distance(final UnwrappableTranslation2d other) {
        return inverse().translateBy(other).norm();
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof UnwrappableTranslation2d))
            return false;
        return distance((UnwrappableTranslation2d) other) < Util.kEpsilon;
    }

    @Override
    public UnwrappableTranslation2d getTranslation() {
        return this;
    }
}
