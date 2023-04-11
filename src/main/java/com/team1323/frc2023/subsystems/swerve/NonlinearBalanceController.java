package com.team1323.frc2023.subsystems.swerve;

import com.team1323.frc2023.Settings;
import com.team1323.lib.math.TwoPointRamp;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class NonlinearBalanceController implements BalanceController {
    private static final double kPitchDeadband = 5.0;

    private final TwoPointRamp outputRamp = Settings.kIsUsingCompBot ?
    new TwoPointRamp(
        new Translation2d(kPitchDeadband, 0.03),
        new Translation2d(16.0, 0.3),
        2.0,
        true
    ) :
    new TwoPointRamp(
        new Translation2d(kPitchDeadband, 0.03),
        new Translation2d(16.0, 0.25),
        2.0,
        true
    );

    private Rotation2d targetPitch = Rotation2d.identity();
    private double outputScalar = 1.0;

    @Override
    public void start(Rotation2d targetPitch) {
        this.targetPitch = targetPitch;
        outputScalar = 1.0;
    }

    @Override
    public Translation2d update(Rotation2d robotPitch, double timestamp) {
        final double error = Math.toDegrees(targetPitch.distance(robotPitch));
        double output = 0.0;
        if (Math.abs(error) < 4.0) {
            outputScalar = 0.5;
        }
        if (Math.abs(error) >= kPitchDeadband) {
            output = Math.signum(error) * outputRamp.calculate(Math.abs(error));
            output *= outputScalar;
        }

        return new Translation2d(output, 0.0);
    }

    @Override
    public boolean isOnTarget() {
        return false;
    }
    
}
