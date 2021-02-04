package com.team1323.lib.geometry;

public interface IUnwrappablePose2d<S> extends IUnwrappableRotation2d<S>, IUnwrappableTranslation2d<S> {
    public UnwrappablePose2d getPose();

    public S transformBy(UnwrappablePose2d transform);

    public S mirror();
}
