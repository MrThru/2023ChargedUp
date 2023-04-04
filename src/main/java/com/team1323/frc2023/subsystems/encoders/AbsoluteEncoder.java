package com.team1323.frc2023.subsystems.encoders;

public interface AbsoluteEncoder {
    public double getDegrees();

    public void setPosition(double degrees);
}
