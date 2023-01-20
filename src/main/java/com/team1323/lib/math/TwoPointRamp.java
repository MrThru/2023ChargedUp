package com.team1323.lib.math;

import com.team254.lib.geometry.Translation2d;

public class TwoPointRamp {
    private final Translation2d lowPoint, highPoint;
    private final double power;
    private final boolean maxOutAtHighPoint;

    public TwoPointRamp(Translation2d lowPoint, Translation2d highPoint, double power, boolean maxOutAtHighPoint) {
        this.lowPoint = lowPoint;
        this.highPoint = highPoint;
        this.power = power;
        this.maxOutAtHighPoint = maxOutAtHighPoint;
    }

    public double calculate(double input) {
        if (input < lowPoint.x()) {
            return lowPoint.y();
        }

        if (input > highPoint.x() && maxOutAtHighPoint) {
            return highPoint.y();
        }

        double normalizedInput = (input - lowPoint.x()) / (highPoint.x() - lowPoint.x());
        double normalizedOutput = Math.pow(normalizedInput, power);
        double output = lowPoint.y() + (normalizedOutput * (highPoint.y() - lowPoint.y()));

        return output;
    }
}
