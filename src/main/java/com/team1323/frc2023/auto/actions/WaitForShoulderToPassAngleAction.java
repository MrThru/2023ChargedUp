// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.auto.actions;

import com.team1323.frc2023.subsystems.Shoulder;
import com.team1323.lib.util.Stopwatch;

/** Add your docs here. */
public class WaitForShoulderToPassAngleAction implements Action {
    private final double timeoutSeconds;
    private final Stopwatch timeoutStopwatch = new Stopwatch();
    private final Shoulder shoulder;
    private final double targetAngle;
    private double startingAngle = 0;
    private boolean leftSide = false;

    public WaitForShoulderToPassAngleAction(double targetAngle) {
        this(targetAngle, 3.0);
    }

    public WaitForShoulderToPassAngleAction(double targetAngle, double timeoutSeconds) {
        shoulder = Shoulder.getInstance();
        this.targetAngle = targetAngle;
        this.timeoutSeconds = timeoutSeconds;
    }

    @Override
    public boolean isFinished() {
        return ((Math.signum(this.targetAngle - shoulder.getPosition()) == 1) != leftSide) || timeoutStopwatch.getTime() >= timeoutSeconds;
    }

    @Override
    public void start() {
        timeoutStopwatch.start();
        startingAngle = shoulder.getPosition();
        leftSide = Math.signum(this.targetAngle - startingAngle) == 1;
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
