package com.team1323.lib.math;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

public class Units {
    private static final double kInchesPerMeter = 39.37007874;

    public static double inchesToMeters(double inches) {
        return inches / kInchesPerMeter;
    }

    public static double metersToInches(double meters) {
        return meters * kInchesPerMeter;
    }

    public static Translation2d inchesToMeters(Translation2d translationInches) {
        return new Translation2d(inchesToMeters(translationInches.x()), inchesToMeters(translationInches.y()));
    }

    public static Pose2d inchesToMeters(Pose2d poseInches) {
        return new Pose2d(inchesToMeters(poseInches.getTranslation()), poseInches.getRotation());
    }

    public static Translation2d metersToInches(Translation2d translationMeters) {
        return new Translation2d(metersToInches(translationMeters.x()), metersToInches(translationMeters.y()));
    }

    public static Pose2d metersToInches(Pose2d poseMeters) {
        return new Pose2d(metersToInches(poseMeters.getTranslation()), poseMeters.getRotation());
    }
}
