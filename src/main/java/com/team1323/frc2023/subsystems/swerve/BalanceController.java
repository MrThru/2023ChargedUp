package com.team1323.frc2023.subsystems.swerve;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public interface BalanceController {
    public void start(Rotation2d targetPitch);

    public Translation2d update(Rotation2d robotPitch, double timestamp);

    public boolean isOnTarget();
}
