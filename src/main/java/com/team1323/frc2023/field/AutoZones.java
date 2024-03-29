package com.team1323.frc2023.field;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.TimedState;

public class AutoZones {
    public static final double kXMirror = 325.625;
    public static final double kYMirror = 108.19;

    public static NodeLocation mirror(NodeLocation bottomLeftLocation, Quadrant quadrant) {
        switch (quadrant) {
            case TOP_LEFT:
                return bottomLeftLocation.mirrorAboutY();
            case TOP_RIGHT:
                return bottomLeftLocation.mirrorAboutX().mirrorAboutY();
            case BOTTOM_RIGHT:
                return bottomLeftLocation.mirrorAboutX();
            default:
                return bottomLeftLocation;
        }
    }

    public static Rotation2d mirror(Rotation2d bottomLeftRotation, Quadrant quadrant) {
        switch (quadrant) {
            case TOP_LEFT:
                return bottomLeftRotation.mirrorAboutY();
            case TOP_RIGHT:
                return bottomLeftRotation.mirrorAboutX().mirrorAboutY();
            case BOTTOM_RIGHT:
                return bottomLeftRotation.mirrorAboutX();
            default:
                return bottomLeftRotation;
        }
    }

    public static Translation2d mirror(Translation2d bottomLeftTranslation, Quadrant quadrant) {
        switch (quadrant) {
            case TOP_LEFT:
                return bottomLeftTranslation.mirrorAboutY(kYMirror);
            case TOP_RIGHT:
                return bottomLeftTranslation.mirrorAboutX(kXMirror).mirrorAboutY(kYMirror);
            case BOTTOM_RIGHT:
                return bottomLeftTranslation.mirrorAboutX(kXMirror);
            default:
                return bottomLeftTranslation;
        }
    }

    public static Pose2d mirror(Pose2d bottomLeftPose, Quadrant quadrant) {
        return new Pose2d(mirror(bottomLeftPose.getTranslation(), quadrant), mirror(bottomLeftPose.getRotation(), quadrant));
    }

    public static Trajectory<TimedState<Pose2dWithCurvature>> mirror(Trajectory<TimedState<Pose2dWithCurvature>> bottomLeftTrajectory, 
            Quadrant quadrant) {
        double defaultVelocity = bottomLeftTrajectory.defaultVelocity();
        switch (quadrant) {
            case TOP_LEFT:
                return TrajectoryUtil.mirrorAboutYTimed(bottomLeftTrajectory, kYMirror, defaultVelocity);
            case TOP_RIGHT:
                Trajectory<TimedState<Pose2dWithCurvature>> xMirroredTrajectory = 
                        TrajectoryUtil.mirrorAboutXTimed(bottomLeftTrajectory, kXMirror, defaultVelocity);
                return TrajectoryUtil.mirrorAboutYTimed(xMirroredTrajectory, kYMirror, defaultVelocity);
            case BOTTOM_RIGHT:
                return TrajectoryUtil.mirrorAboutXTimed(bottomLeftTrajectory, kXMirror, defaultVelocity);
            default:
                return bottomLeftTrajectory;
        }
    }

    public enum StartingSide {
        LEFT, RIGHT;
    }

    public enum Quadrant {
        BOTTOM_LEFT, TOP_LEFT, TOP_RIGHT, BOTTOM_RIGHT;

        public boolean hasBump() {
            return this == BOTTOM_LEFT || this == BOTTOM_RIGHT;
        }
    }
}
