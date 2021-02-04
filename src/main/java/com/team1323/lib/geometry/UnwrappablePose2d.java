package com.team1323.lib.geometry;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.Util;

/**
 * Represents a 2d pose (rigid transform) containing translational and rotational elements.
 * <p>
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
public class UnwrappablePose2d implements IUnwrappablePose2d<UnwrappablePose2d> {
    protected static final UnwrappablePose2d kIdentity = new UnwrappablePose2d();

    public static final UnwrappablePose2d identity() {
        return kIdentity;
    }

    private final static double kEps = 1E-9;

    protected final UnwrappableTranslation2d translation_;
    protected final UnwrappableRotation2d rotation_;

    public UnwrappablePose2d() {
        translation_ = new UnwrappableTranslation2d();
        rotation_ = new UnwrappableRotation2d();
    }

    public UnwrappablePose2d(double x, double y, final UnwrappableRotation2d rotation) {
        translation_ = new UnwrappableTranslation2d(x, y);
        rotation_ = rotation;
    }

    public UnwrappablePose2d(final UnwrappableTranslation2d translation, final UnwrappableRotation2d rotation) {
        translation_ = translation;
        rotation_ = rotation;
    }

    public UnwrappablePose2d(final UnwrappablePose2d other) {
        translation_ = new UnwrappableTranslation2d(other.translation_);
        rotation_ = new UnwrappableRotation2d(other.rotation_);
    }

    public static UnwrappablePose2d fromTranslation(final UnwrappableTranslation2d translation) {
        return new UnwrappablePose2d(translation, new UnwrappableRotation2d());
    }

    public static UnwrappablePose2d fromRotation(final UnwrappableRotation2d rotation) {
        return new UnwrappablePose2d(new UnwrappableTranslation2d(), rotation);
    }

    public Pose2d wrap() {
        return new Pose2d(translation_.wrap(), rotation_.wrap());
    }

    /**
     * Transforms the pose by the given transformation and returns the new pose.
     *
     * @param other The transform to transform the pose by.
     * @return The transformed pose.
     */
    public UnwrappablePose2d plus(UnwrappableTransform2d other) {
        return new UnwrappablePose2d(translation_.translateBy(other.getTranslation().rotateBy(rotation_)),
                rotation_.rotateBy(other.getRotation()));
    }

    /**
     * Obtain a new Pose2d from a (constant curvature) velocity.
     *
     * <p>
     * See <a href="https://file.tavsys.net/control/state-space-guide.pdf"> Controls
     * Engineering in the FIRST Robotics Competition</a> section on nonlinear pose
     * estimation for derivation.
     *
     * <p>
     * The twist is a change in pose in the robot's coordinate frame since the
     * previous pose update. When the user runs exp() on the previous known
     * field-relative pose with the argument being the twist, the user will receive
     * the new field-relative pose.
     *
     * <p>
     * "Exp" represents the pose exponential, which is solving a differential
     * equation moving the pose forward in time.
     *
     * @param twist The change in pose in the robot's coordinate frame since the
     *              previous pose update. For example, if a non-holonomic robot
     *              moves forward 0.01 meters and changes angle by 0.5 degrees since
     *              the previous pose update, the twist would be Twist2d{0.01, 0.0,
     *              toRadians(0.5)}
     * @return The new pose of the robot.
     */
    @SuppressWarnings("LocalVariableName")
    public UnwrappablePose2d wpiExp(UnwrappableTwist2d twist) {
        double dx = twist.dx;
        double dy = twist.dy;
        double dtheta = twist.dtheta;

        double sinTheta = Math.sin(dtheta);
        double cosTheta = Math.cos(dtheta);

        double s;
        double c;
        if (Math.abs(dtheta) < 1E-9) {
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            c = 0.5 * dtheta;
        } else {
            s = sinTheta / dtheta;
            c = (1 - cosTheta) / dtheta;
        }
        var transform = new UnwrappableTransform2d(new UnwrappableTranslation2d(dx * s - dy * c, dx * c + dy * s),
                new UnwrappableRotation2d(cosTheta, sinTheta, true));

        return this.plus(transform);
    }

    /**
     * Obtain a new Pose2d from a (constant curvature) velocity. See:
     * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     */
    public static UnwrappablePose2d exp(final UnwrappableTwist2d delta) {
        double sin_theta = Math.sin(delta.dtheta);
        double cos_theta = Math.cos(delta.dtheta);
        double s, c;
        if (Math.abs(delta.dtheta) < kEps) {
            s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
            c = .5 * delta.dtheta;
        } else {
            s = sin_theta / delta.dtheta;
            c = (1.0 - cos_theta) / delta.dtheta;
        }
        return new UnwrappablePose2d(new UnwrappableTranslation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
                new UnwrappableRotation2d(cos_theta, sin_theta, false));
    }

    /**
     * Logical inverse of the above.
     */
    public static UnwrappableTwist2d log(final UnwrappablePose2d transform) {
        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = transform.getRotation().cos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < kEps) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().sin()) / cos_minus_one;
        }
        final UnwrappableTranslation2d translation_part = transform.getTranslation()
                .rotateBy(new UnwrappableRotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta, false));
        return new UnwrappableTwist2d(translation_part.x(), translation_part.y(), dtheta);
    }

    @Override
    public UnwrappableTranslation2d getTranslation() {
        return translation_;
    }

    @Override
    public UnwrappableRotation2d getRotation() {
        return rotation_;
    }

    /**
     * Transforming this RigidTransform2d means first translating by
     * other.translation and then rotating by other.rotation
     *
     * @param other The other transform.
     * @return This transform * other
     */
    @Override
    public UnwrappablePose2d transformBy(final UnwrappablePose2d other) {
        return new UnwrappablePose2d(translation_.translateBy(other.translation_.rotateBy(rotation_)),
                rotation_.rotateBy(other.rotation_));
    }

    /**
     * The inverse of this transform "undoes" the effect of translating by this
     * transform.
     *
     * @return The opposite of this transform.
     */
    public UnwrappablePose2d inverse() {
        UnwrappableRotation2d rotation_inverted = rotation_.inverse();
        return new UnwrappablePose2d(translation_.inverse().rotateBy(rotation_inverted), rotation_inverted);
    }

    public UnwrappablePose2d normal() {
        return new UnwrappablePose2d(translation_, rotation_.normal());
    }

    /**
     * Finds the point where the heading of this pose intersects the heading of
     * another. Returns (+INF, +INF) if parallel.
     */
    public UnwrappableTranslation2d intersection(final UnwrappablePose2d other) {
        final UnwrappableRotation2d other_rotation = other.getRotation();
        if (rotation_.isParallel(other_rotation)) {
            // Lines are parallel.
            return new UnwrappableTranslation2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        if (Math.abs(rotation_.cos()) < Math.abs(other_rotation.cos())) {
            return intersectionInternal(this, other);
        } else {
            return intersectionInternal(other, this);
        }
    }

    /**
     * Return true if this pose is (nearly) colinear with the another.
     */
    public boolean isColinear(final UnwrappablePose2d other) {
        if (!getRotation().isParallel(other.getRotation()))
            return false;
        final UnwrappableTwist2d twist = log(inverse().transformBy(other));
        return (Util.epsilonEquals(twist.dy, 0.0) && Util.epsilonEquals(twist.dtheta, 0.0));
    }

    public boolean epsilonEquals(final UnwrappablePose2d other, double epsilon) {
        return getTranslation().epsilonEquals(other.getTranslation(), epsilon)
                && getRotation().isParallel(other.getRotation());
    }

    private static UnwrappableTranslation2d intersectionInternal(final UnwrappablePose2d a, final UnwrappablePose2d b) {
        final UnwrappableRotation2d a_r = a.getRotation();
        final UnwrappableRotation2d b_r = b.getRotation();
        final UnwrappableTranslation2d a_t = a.getTranslation();
        final UnwrappableTranslation2d b_t = b.getTranslation();

        final double tan_b = b_r.tan();
        final double t = ((a_t.x() - b_t.x()) * tan_b + b_t.y() - a_t.y()) / (a_r.sin() - a_r.cos() * tan_b);
        if (Double.isNaN(t)) {
            return new UnwrappableTranslation2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        return a_t.translateBy(a_r.toTranslation().scale(t));
    }

    /**
     * Do twist interpolation of this pose assuming constant curvature.
     */
    @Override
    public UnwrappablePose2d interpolate(final UnwrappablePose2d other, double x) {
        if (x <= 0) {
            return new UnwrappablePose2d(this);
        } else if (x >= 1) {
            return new UnwrappablePose2d(other);
        }
        final UnwrappableTwist2d twist = UnwrappablePose2d.log(inverse().transformBy(other));
        return transformBy(UnwrappablePose2d.exp(twist.scaled(x)));
    }

    @Override
    public String toString() {
        return "T:" + translation_.toString() + ", R:" + rotation_.toString();
    }

    @Override
    public String toCSV() {
        return translation_.toCSV() + "," + rotation_.toCSV();
    }

    @Override
    public double distance(final UnwrappablePose2d other) {
        return UnwrappablePose2d.log(inverse().transformBy(other)).norm();
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof UnwrappablePose2d))
            return false;
        return epsilonEquals((UnwrappablePose2d) other, Util.kEpsilon);
    }

    @Override
    public UnwrappablePose2d getPose() {
        return this;
    }

    @Override
    public UnwrappablePose2d mirror() {
        return new UnwrappablePose2d(new UnwrappableTranslation2d(getTranslation().x(), -getTranslation().y()),
                getRotation().inverse());
    }
}
