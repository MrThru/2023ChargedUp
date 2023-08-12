package com.team1323.frc2023.auto;

import com.team1323.frc2023.requests.WaitForPrereqRequest;
import com.team1323.frc2023.subsystems.swerve.Swerve;

public class WaitToFinishPathRequest extends WaitForPrereqRequest {
    public WaitToFinishPathRequest(double timeoutSeconds) {
        super(() -> Swerve.getInstance().hasFinishedPath(), timeoutSeconds);
    }
}
