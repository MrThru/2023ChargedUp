package com.team1323.lib.util;

import java.util.Arrays;
import java.util.stream.DoubleStream;

import org.littletonrobotics.junction.Logger;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

public class LogUtil {
    private static final Logger logger = Logger.getInstance();

    public static void recordTranslation2d(String key, Translation2d translation) {
        logger.recordOutput(key, new double[]{ translation.x(), translation.y() });
    }

    public static void recordRotation2d(String key, Rotation2d rotation) {
        logger.recordOutput(key, rotation.getDegrees());
    }

    public static void recordPose2d(String key, Pose2d... poses) {
        final double[] doubleArray = Arrays.stream(poses)
                .flatMapToDouble(pose -> DoubleStream.of(pose.getTranslation().x(), pose.getTranslation().y(), pose.getRotation().getRadians()))
                .toArray();
        logger.recordOutput(key, doubleArray);
    }

    public static void recordTrajectory(String key, Trajectory<TimedState<Pose2dWithCurvature>> trajectory) {
        final Pose2d[] poses = new Pose2d[trajectory.length()];
        for (int i = 0; i < trajectory.length(); i++) {
            poses[i] = trajectory.getState(i).state().getPose();
        }
        recordPose2d(key, poses);
    }
}
