package com.team254.lib.geometry;

public interface IPose2d<S> extends IRotation2d<S>, ITranslation2d<S> {
    public Pose2d getPose();

    public S transformBy(Pose2d transform);

    public S mirror();

    /**
     * Mirrors a pose about the vertical line defined by x = xValue
     */
    public S mirrorAboutX(double xValue);

    /**
     * Mirros a pose about the horizontal line defined by y = yValue
     */
    public S mirrorAboutY(double yValue);
}
