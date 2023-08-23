package com.team1323.frc2023.vision;

import java.util.Arrays;
import java.util.List;

import com.team1323.frc2023.Settings;
import com.team1323.frc2023.subsystems.Subsystem;
import com.team1323.frc2023.vision.Limelight.Pipeline;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.Timer;

public class LimelightManager extends Subsystem {
    private static LimelightManager instance = null;
    public static LimelightManager getInstance() {
        if (instance == null) {
            instance = new LimelightManager();
        }
        return instance;
    }

    private final Limelight centerLimelight, leftLimelight, rightLimelight;
    private final List<Limelight> allLimelights;
    
    private ProcessingMode processingMode = ProcessingMode.CENTER_FIDUCIAL;

    private LimelightManager() {
        centerLimelight = new Limelight(
            "Limelight", 
            new Translation2d(3.86, 0.0), 
            52.614, 
            Rotation2d.fromDegrees(Settings.kIsUsingCompBot ? 29.5 : 29.5), 
            Rotation2d.fromDegrees(0.0)
        );
        leftLimelight = new Limelight(
            "Left Limelight", 
            new Translation2d(0.0, 0.0), 
            0.0, 
            Rotation2d.fromDegrees(0.0), 
            Rotation2d.fromDegrees(0.0)
        );
        rightLimelight = new Limelight(
            "Right Limelight", 
            new Translation2d(0.0, 0.0), 
            0.0, 
            Rotation2d.fromDegrees(0.0), 
            Rotation2d.fromDegrees(0.0)
        );

        allLimelights = Arrays.asList(centerLimelight, leftLimelight, rightLimelight);
    }

    public void setProcessingMode(ProcessingMode mode) {
        processingMode = mode;
        switch (processingMode) {
            case LEFT_AND_RIGHT_FIDUCIAL:
                leftLimelight.setPipeline(Pipeline.FIDUCIAL);
                rightLimelight.setPipeline(Pipeline.FIDUCIAL);
                break;
            case CENTER_FIDUCIAL:
                centerLimelight.setPipeline(Pipeline.FIDUCIAL);
                break;
            case CENTER_RETRO:
                centerLimelight.setPipeline(Pipeline.RETRO);
                break;
            case CENTER_DETECTOR:
                centerLimelight.setPipeline(Pipeline.DETECTOR);
                break;
        }
    }

    public boolean areActiveLimelightsConnected() {
        switch (processingMode) {
            case LEFT_AND_RIGHT_FIDUCIAL:
                return leftLimelight.isConnected() || rightLimelight.isConnected();
            default:
                return centerLimelight.isConnected();
        }
    }

    public Limelight getCenterLimelight() {
        return centerLimelight;
    }

    public Pose2d getRobotConePickupPosition(Translation2d trueConePosition) {
        return centerLimelight.getRobotConePickupPosition(trueConePosition);
    }

    @Override
    public void readPeriodicInputs() {
        allLimelights.forEach(Limelight::readPeriodicInputs);

        double timestamp = Timer.getFPGATimestamp();
        switch (processingMode) {
            case LEFT_AND_RIGHT_FIDUCIAL:
                leftLimelight.processAllTargets(timestamp);
                rightLimelight.processAllTargets(timestamp);
                break;
            default:
                centerLimelight.processAllTargets(timestamp);
                break;
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public void outputTelemetry() {
        allLimelights.forEach(Limelight::outputTelemetry);
    }

    public enum ProcessingMode {
        CENTER_FIDUCIAL,
        CENTER_RETRO,
        CENTER_DETECTOR,
        LEFT_AND_RIGHT_FIDUCIAL
    }
}
