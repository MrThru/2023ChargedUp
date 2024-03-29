// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.auto.actions;

import com.team1323.frc2023.subsystems.Tunnel;
import com.team1323.lib.util.Stopwatch;

/** Add your docs here. */
public class WaitToIntakeCubeAction implements Action {
    private final double timeoutSeconds;
    private final Stopwatch timeoutStopwatch = new Stopwatch();
    private final Tunnel tunnel;
    private boolean useFirstBanner = false;

    public WaitToIntakeCubeAction(double timeoutSeconds) {
        this.timeoutSeconds = timeoutSeconds;
        tunnel = Tunnel.getInstance();
    }
    public WaitToIntakeCubeAction(double timeoutSeconds, boolean useFirstBanner) {
        this(timeoutSeconds);
        this.useFirstBanner = useFirstBanner;
    }

    @Override
    public boolean isFinished() {
        return timeoutStopwatch.getTime() >= timeoutSeconds || (tunnel.getRearBanner() || tunnel.getFrontBanner() || (useFirstBanner && tunnel.getCubeIntakeBanner()));
    }

    @Override
    public void start() {
        timeoutStopwatch.start();
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        
    }

}
