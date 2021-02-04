package com.team1323.lib.geometry;

import com.team254.lib.util.Util;

import java.text.DecimalFormat;

public class UnwrappablePose2dWithCurvature
        implements IUnwrappablePose2d<UnwrappablePose2dWithCurvature>, IUnwrappableCurvature<UnwrappablePose2dWithCurvature> {
    protected static final UnwrappablePose2dWithCurvature kIdentity = new UnwrappablePose2dWithCurvature();

    public static final UnwrappablePose2dWithCurvature identity() {
        return kIdentity;
    }

    protected final UnwrappablePose2d pose_;
    protected final double curvature_;
    protected final double dcurvature_ds_;

    public UnwrappablePose2dWithCurvature() {
        pose_ = new UnwrappablePose2d();
        curvature_ = 0.0;
        dcurvature_ds_ = 0.0;
    }

    public UnwrappablePose2dWithCurvature(final UnwrappablePose2d pose, double curvature) {
        pose_ = pose;
        curvature_ = curvature;
        dcurvature_ds_ = 0.0;
    }

    public UnwrappablePose2dWithCurvature(final UnwrappablePose2d pose, double curvature, double dcurvature_ds) {
        pose_ = pose;
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }

    public UnwrappablePose2dWithCurvature(final UnwrappableTranslation2d translation, final UnwrappableRotation2d rotation,
            double curvature) {
        pose_ = new UnwrappablePose2d(translation, rotation);
        curvature_ = curvature;
        dcurvature_ds_ = 0.0;
    }

    public UnwrappablePose2dWithCurvature(final UnwrappableTranslation2d translation, final UnwrappableRotation2d rotation, double curvature,
            double dcurvature_ds) {
        pose_ = new UnwrappablePose2d(translation, rotation);
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }

    @Override
    public final UnwrappablePose2d getPose() {
        return pose_;
    }

    @Override
    public UnwrappablePose2dWithCurvature transformBy(UnwrappablePose2d transform) {
        return new UnwrappablePose2dWithCurvature(getPose().transformBy(transform), getCurvature(), getDCurvatureDs());
    }

    @Override
    public UnwrappablePose2dWithCurvature mirror() {
        return new UnwrappablePose2dWithCurvature(getPose().mirror().getPose(), -getCurvature(), -getDCurvatureDs());
    }

    @Override
    public double getCurvature() {
        return curvature_;
    }

    @Override
    public double getDCurvatureDs() {
        return dcurvature_ds_;
    }

    @Override
    public final UnwrappableTranslation2d getTranslation() {
        return getPose().getTranslation();
    }

    @Override
    public final UnwrappableRotation2d getRotation() {
        return getPose().getRotation();
    }

    @Override
    public UnwrappablePose2dWithCurvature interpolate(final UnwrappablePose2dWithCurvature other, double x) {
        return new UnwrappablePose2dWithCurvature(getPose().interpolate(other.getPose(), x),
                Util.interpolate(getCurvature(), other.getCurvature(), x),
                Util.interpolate(getDCurvatureDs(), other.getDCurvatureDs(), x));
    }

    @Override
    public double distance(final UnwrappablePose2dWithCurvature other) {
        return getPose().distance(other.getPose());
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof UnwrappablePose2dWithCurvature))
            return false;
        UnwrappablePose2dWithCurvature p2dwc = (UnwrappablePose2dWithCurvature) other;
        return getPose().equals(p2dwc.getPose()) && Util.epsilonEquals(getCurvature(), p2dwc.getCurvature()) && Util.epsilonEquals(getDCurvatureDs(), p2dwc.getDCurvatureDs());
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return getPose().toString() + ", curvature: " + fmt.format(getCurvature()) + ", dcurvature_ds: " + fmt.format(getDCurvatureDs());
    }

    @Override
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return getPose().toCSV() + "," + fmt.format(getCurvature()) + "," + fmt.format(getDCurvatureDs());
    }
}
