package com.team1323.frc2023.subsystems.swerve;

import com.team1323.frc2023.Settings;
import com.team1323.lib.math.TwoPointRamp;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class NonlinearBalanceController implements BalanceController {
    private static final double kPitchDeadband = 7.0;

    private final TwoPointRamp outputRamp = Settings.kIsUsingCompBot ?
    new TwoPointRamp(
        new Translation2d(kPitchDeadband, 0.03),
        new Translation2d(16.0, 0.22),
        2.0,
        true
    ) :
    new TwoPointRamp(
        new Translation2d(kPitchDeadband, 0.03),
        new Translation2d(16.0, 0.2),
        2.0,
        true
    );

    private Rotation2d targetPitch = Rotation2d.identity();

    @Override
    public void start(Rotation2d targetPitch) {
        this.targetPitch = targetPitch;
    }

    @Override
    public Translation2d update(Rotation2d robotPitch, double timestamp) {
        final double error = Math.toDegrees(targetPitch.distance(robotPitch));
        double output = 0.0;
        if (Math.abs(error) >= kPitchDeadband) {
            output = Math.signum(error) * outputRamp.calculate(Math.abs(error));
        }

        return new Translation2d(output, 0.0);
    }

    @Override
    public boolean isOnTarget() {
        return false;
    }
    
}
