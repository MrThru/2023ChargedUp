// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.auto.actions;

import com.team1323.frc2023.subsystems.Tunnel;
import com.team1323.lib.util.Stopwatch;

/** Add your docs here. */
public class WaitToIntakeCubeAction implements Action {
    Stopwatch timeout = new Stopwatch();

    Tunnel tunnel;
    public WaitToIntakeCubeAction() {
        tunnel = Tunnel.getInstance();
    }

    @Override
    public boolean isFinished() {
        return timeout.getTime() > 5 || (tunnel.getRearBanner() || tunnel.getFrontBanner());
    }

    @Override
    public void start() {
        timeout.start();
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
