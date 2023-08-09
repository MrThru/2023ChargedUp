package com.team1323.frc2023.requests.auto;

import com.team1323.frc2023.requests.WaitForPrereqRequest;
import com.team1323.frc2023.subsystems.Claw;

public class WaitToIntakeRequest extends WaitForPrereqRequest {
    public WaitToIntakeRequest(Claw.HoldingObject holdingObject, double timeoutSeconds) {
        super(() -> Claw.getInstance().getCurrentHoldingObject() == holdingObject, timeoutSeconds);
    }
}
