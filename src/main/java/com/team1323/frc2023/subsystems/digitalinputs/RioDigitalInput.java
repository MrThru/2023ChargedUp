package com.team1323.frc2023.subsystems.digitalinputs;

import edu.wpi.first.wpilibj.DigitalInput;

public class RioDigitalInput implements IDigitalInput {
    private final DigitalInput input;

    public RioDigitalInput(int channel) {
        input = new DigitalInput(channel);
    }

    @Override
    public boolean get() {
        return input.get();
    }
}
