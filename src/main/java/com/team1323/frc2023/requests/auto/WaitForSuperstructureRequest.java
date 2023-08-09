package com.team1323.frc2023.requests.auto;

import com.team1323.frc2023.requests.WaitForPrereqRequest;
import com.team1323.frc2023.subsystems.superstructure.Superstructure;

public class WaitForSuperstructureRequest extends WaitForPrereqRequest {
    public WaitForSuperstructureRequest(double timeoutSeconds) {
        super(() -> Superstructure.getInstance().areAllRequestsCompleted(), timeoutSeconds);
    }
}
