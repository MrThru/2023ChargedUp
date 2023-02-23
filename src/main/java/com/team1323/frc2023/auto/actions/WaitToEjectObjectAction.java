// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.auto.actions;

import com.team1323.frc2023.subsystems.Claw;
import com.team1323.lib.util.Stopwatch;

/** Add your docs here. */
public class WaitToEjectObjectAction implements Action {

    Claw claw;
    Stopwatch stopwatch = new Stopwatch();
    public WaitToEjectObjectAction() {
        claw = Claw.getInstance();
    }

    @Override
    public boolean isFinished() {
        return claw.getCurrentHoldingObject() == Claw.HoldingObject.None || stopwatch.getTime() > 2.0;
    }

    @Override
    public void start() {
        stopwatch.startIfNotRunning();
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
