package com.team1323.lib.util;

import org.littletonrobotics.junction.Logger;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class LogUtil {
    private static final Logger logger = Logger.getInstance();

    public static void recordTranslation2d(String key, Translation2d translation) {
        logger.recordOutput(key, new double[]{ translation.x(), translation.y() });
    }

    public static void recordRotation2d(String key, Rotation2d rotation) {
        logger.recordOutput(key, rotation.getDegrees());
    }

    public static void recordPose2d(String key, Pose2d pose) {
        logger.recordOutput(key, new double[]{ pose.getTranslation().x(), pose.getTranslation().y(), pose.getRotation().getRadians() });
    }
}
