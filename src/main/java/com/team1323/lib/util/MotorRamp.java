package com.team1323.lib.util;

import com.team1323.frc2023.Constants;

import edu.wpi.first.wpilibj.Timer;

public class MotorRamp {
    private double setpoint;
    private final double max;
    private final double min;
    private double rampRate;

    private double lastTimestamp = 0.0;

    private enum Direction {
        RAMP_UP, RAMP_DOWN, CONSTANT
    }
    Direction direction = Direction.CONSTANT;

    public MotorRamp(double min, double max, double secondsFromZeroToMax) {
        this.min = min;
        this.max = max;
        if (Util.epsilonEquals(secondsFromZeroToMax, 0.0))
            rampRate = max * (1.0 / Constants.kLooperDt);
        else
            rampRate = max / secondsFromZeroToMax;
        setSetpoint(0.0, 0.0);
    }

    public MotorRamp(double max, double secondsFromZeroToMax) {
        this(-max, max, secondsFromZeroToMax);
    }

    public void reset(double timestamp) {
        setpoint = 0.0;
        direction = Direction.CONSTANT;
        lastTimestamp = timestamp;
    }

    public void setSetpoint(double setpoint, double motorSpeed) {
        this.setpoint = Util.limit(setpoint, min, max);

        if (setpoint > motorSpeed)
            direction = Direction.RAMP_UP;
        else if (setpoint < motorSpeed)
            direction = Direction.RAMP_DOWN;
        else
            direction = Direction.CONSTANT;

        lastTimestamp = Timer.getFPGATimestamp();
    }

    public double update(double motorSpeed, double timestamp) {
        double output = setpoint;
        switch(direction) {
            case RAMP_UP:
                double increment = (timestamp - lastTimestamp) * rampRate;
                if (motorSpeed + increment > setpoint)
                    output = setpoint;
                else
                    output = motorSpeed + increment;
                break;
            case RAMP_DOWN:
                double decrement = -((timestamp - lastTimestamp) * rampRate);
                if (motorSpeed + decrement < setpoint)
                    output = setpoint;
                else
                    output = motorSpeed + decrement;
                break;
            case CONSTANT:
                output = setpoint;
                break;
            default:
                break;
        }
        if (Util.epsilonEquals(output, setpoint))
            direction = Direction.CONSTANT;
        lastTimestamp = timestamp;

        return output;
    }
}