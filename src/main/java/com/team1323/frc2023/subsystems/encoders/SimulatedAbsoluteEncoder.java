package com.team1323.frc2023.subsystems.encoders;

public class SimulatedAbsoluteEncoder implements AbsoluteEncoder {

    @Override
    public double getDegrees() {
        return 0.0;
    }

    @Override
    public void setPosition(double degrees) {
    }
    
    @Override
    public boolean isConnected() {
        return true;
    }

}
