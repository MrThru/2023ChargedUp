// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.auto.actions;

import com.team1323.frc2023.subsystems.swerve.Swerve;

/** Add your docs here. */
public class WaitToBalanceAction implements Action {
    Swerve swerve;

    public WaitToBalanceAction() {
        swerve = Swerve.getInstance();
    }
    @Override
    public boolean isFinished() {
        return swerve.balancePIDOnTarget();
    }

    @Override
    public void start() {
        // TODO Auto-generated method stub
        
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
