package com.team1323.frc2023.requests.auto;

import com.team1323.frc2023.requests.WaitForPrereqRequest;
import com.team1323.frc2023.subsystems.Tunnel;

public class WaitToIntakeCubeRequest extends WaitForPrereqRequest {
    public WaitToIntakeCubeRequest(boolean useFirstBanner, double timeoutSeconds) {
        super(() -> {
            return Tunnel.getInstance().getRearBanner() || Tunnel.getInstance().getFrontBanner() ||
                    (useFirstBanner && Tunnel.getInstance().getCubeIntakeBanner());
        }, timeoutSeconds);
    }
}
