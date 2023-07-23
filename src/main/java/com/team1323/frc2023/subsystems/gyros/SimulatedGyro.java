package com.team1323.frc2023.subsystems.gyros;

public class SimulatedGyro implements Gyro {

    @Override
    public void setYaw(double angle) {
    }

    @Override
    public double getYaw() {
        return 0.0;
    }

    @Override
    public double getPitch() {
        return 0.0;
    }

    @Override
    public double getRoll() {
        return 0.0;
    }

    @Override
    public double[] getYPR() {
        return new double[]{0.0, 0.0, 0.0};
    }

    @Override
    public void resetRoll() {
    }
    
}
