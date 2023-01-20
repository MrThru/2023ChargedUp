package com.team1323.lib.util;

import edu.wpi.first.wpilibj.Timer;

public class Stopwatch {
    private double startTime = Double.POSITIVE_INFINITY;

    public void start() {
        startTime = Timer.getFPGATimestamp();
    }

    public void startIfNotRunning() {
        if (Double.isInfinite(startTime)) {
            start();
        }
    }

    public double getTime() {
        if (Double.isInfinite(startTime)) {
            return 0.0;
        }

        return Timer.getFPGATimestamp() - startTime;
    }

    public void reset() {
        startTime = Double.POSITIVE_INFINITY;
    }
}
