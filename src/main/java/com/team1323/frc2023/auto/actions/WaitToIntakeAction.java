// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.auto.actions;

import com.team1323.frc2023.subsystems.Claw;
import com.team1323.lib.util.Stopwatch;

/** Add your docs here. */
public class WaitToIntakeAction implements Action {
    private final Claw claw;
    private final double timeoutSeconds;
    private final Stopwatch timeoutStopwatch = new Stopwatch();
    private final Claw.HoldingObject targetObject;

    public WaitToIntakeAction(Claw.HoldingObject holdingObject) {
        this(holdingObject, 3.0);
    }

    public WaitToIntakeAction(Claw.HoldingObject holdingObject, double timeoutSeconds) {
        targetObject = holdingObject;
        this.timeoutSeconds = timeoutSeconds;
        claw = Claw.getInstance();
    }

    @Override
    public boolean isFinished() {
        return (claw.getCurrentHoldingObject() == targetObject || timeoutStopwatch.getTime() > timeoutSeconds);
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
