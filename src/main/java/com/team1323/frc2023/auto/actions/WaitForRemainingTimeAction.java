package com.team1323.frc2023.auto.actions;

import edu.wpi.first.wpilibj.Timer;

public class WaitForRemainingTimeAction implements Action {
    private final double targetRemainingTime, autoStartTime;

    public WaitForRemainingTimeAction(double targetRemainingTime, double autoStartTime) {
        this.targetRemainingTime = targetRemainingTime;
        this.autoStartTime = autoStartTime;
    }

    @Override
    public boolean isFinished() {
        double elapsedTime = Timer.getFPGATimestamp() - autoStartTime;
        return (15.0 - elapsedTime) <= targetRemainingTime;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        
    }

    @Override
    public void done() {
        
    }
}
