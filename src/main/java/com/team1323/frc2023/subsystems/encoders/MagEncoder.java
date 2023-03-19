package com.team1323.frc2023.subsystems.encoders;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class MagEncoder implements AbsoluteEncoder {
    private final DutyCycle encoder;
    private final double readingSign;

    public MagEncoder(int digitalInputChannel, boolean isReversed) {
        encoder = new DutyCycle(new DigitalInput(digitalInputChannel));
        readingSign = isReversed ? -1.0 : 1.0;
    }

    @Override
    public double getDegrees() {
        return readingSign * encoder.getOutput() * 360.0;
    }

    @Override
    public void setPosition(double position) {
        // no-op
    }
}
