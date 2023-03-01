// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.auto.actions;

import com.team1323.frc2023.subsystems.Shoulder;

/** Add your docs here. */
public class WaitForShoulderToPassAngleAction implements Action {
    Shoulder shoulder;
    double startingAngle = 0;
    double targetAngle = 0;
    boolean leftSide = false;
    public WaitForShoulderToPassAngleAction(double targetAngle) {
        shoulder = Shoulder.getInstance();
        this.targetAngle = targetAngle;
    }
    @Override
    public boolean isFinished() {
        return (Math.signum(this.targetAngle - shoulder.getPosition()) == 1) != leftSide;
    }

    @Override
    public void start() {
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
