package com.team1323.frc2023.auto;

import com.team1323.frc2023.requests.Request;
import com.team1323.lib.util.Stopwatch;

public class WaitForRemainingTimeRequest extends Request {
    private static final double kAutoDurationSeconds = 15.0;

    private final double targetRemainingTime;
    private final Stopwatch runtimeStopwatch;

    public WaitForRemainingTimeRequest(double targetRemainingTime, Stopwatch runtimeStopwatch) {
        this.targetRemainingTime = targetRemainingTime;
        this.runtimeStopwatch = runtimeStopwatch;
    }

    @Override
    public void act() {
    }

    @Override
    public boolean isFinished() {
        return (kAutoDurationSeconds - runtimeStopwatch.getTime()) <= targetRemainingTime;
    }
}
