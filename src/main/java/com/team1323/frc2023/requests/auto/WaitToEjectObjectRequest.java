package com.team1323.frc2023.requests.auto;

import com.team1323.frc2023.requests.WaitForPrereqRequest;
import com.team1323.frc2023.subsystems.Claw;

public class WaitToEjectObjectRequest extends WaitForPrereqRequest {
    public WaitToEjectObjectRequest(double timeoutSeconds) {
        super(() -> Claw.getInstance().getCurrentHoldingObject() == Claw.HoldingObject.None, timeoutSeconds);
    }
}
