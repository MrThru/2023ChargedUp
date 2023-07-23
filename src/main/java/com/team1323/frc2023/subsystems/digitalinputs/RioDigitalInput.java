package com.team1323.frc2023.subsystems.digitalinputs;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;

public class RioDigitalInput implements IDigitalInput {
    private final DigitalInput input;

    public static IDigitalInput createRealOrSimulatedInput(int channel) {
        return RobotBase.isReal() ? new RioDigitalInput(channel) : new SimulatedDigitalInput();
    }

    private RioDigitalInput(int channel) {
        input = new DigitalInput(channel);
    }

    @Override
    public boolean get() {
        return input.get();
    }
}
