// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.auto.actions;

import com.team1323.frc2023.subsystems.Claw;
import com.team1323.lib.util.Stopwatch;

/** Add your docs here. */
public class WaitToIntakeAction implements Action {

    Claw claw;
    Claw.HoldingObject targetObject = Claw.HoldingObject.None;
    Stopwatch timeout = new Stopwatch();

    public WaitToIntakeAction(Claw.HoldingObject holdingObject) {
        targetObject = holdingObject;
    }
    @Override
    public boolean isFinished() {
        return (Claw.getInstance().getCurrentHoldingObject() == targetObject || timeout.getTime() > 3.0);
    }

    @Override
    public void start() {
        timeout.startIfNotRunning();
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
