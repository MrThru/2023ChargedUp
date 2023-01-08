package com.team1323.lib.math;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.math.geometry.Translation2d;

public class Units {
    private static final double kInchesPerMeter = 39.37007874;

    public static double inchesToMeters(double inches) {
        return inches / kInchesPerMeter;
    }

    public static double metersToInches(double meters) {
        return meters * kInchesPerMeter;
    }

    public static Translation2d inchesToMeters(com.team254.lib.geometry.Translation2d translationInches) {
        return new Translation2d(inchesToMeters(translationInches.x()), inchesToMeters(translationInches.y()));
    }

    public static edu.wpi.first.math.geometry.Pose2d inchesToMeters(Pose2d poseInches) {
        return new edu.wpi.first.math.geometry.Pose2d(inchesToMeters(poseInches.getTranslation()), edu.wpi.first.math.geometry.Rotation2d.fromDegrees(poseInches.getRotation().getDegrees()));
    }

    public static com.team254.lib.geometry.Translation2d metersToInches(Translation2d translationMeters) {
        return new com.team254.lib.geometry.Translation2d(metersToInches(translationMeters.getX()), metersToInches(translationMeters.getY()));
    }

    public static Pose2d metersToInches(edu.wpi.first.math.geometry.Pose2d poseMeters) {
        return new Pose2d(metersToInches(poseMeters.getTranslation()), Rotation2d.fromDegrees(poseMeters.getRotation().getDegrees()));
    }
}
