// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2023.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.Settings;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.lib.util.Stopwatch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class LEDs extends Subsystem {
    CANdle candle;

    public int mRed = 0;
    public int mGreen = 0;
    public int mBlue = 0;

    private static LEDs instance = null;

    public static LEDs getInstance() {
        if (instance == null)
            instance = new LEDs();
        return instance;
    }

    public LEDs() {
        candle = new CANdle(Ports.CANDLE);
        candle.configLEDType(LEDStripType.GRB);
    }

    public enum LEDMode {
        SOLID, BLINK_SOLID, RAINBOW, FIRE, TWINKLE, STROBE;

    }

    private LEDMode selectedLEDType = LEDMode.SOLID;

    public LEDMode getLEDType() {
        return selectedLEDType;
    }

    private LEDColors selectedLEDColors = LEDColors.OFF;

    public LEDColors getSelectedLEDColors() {
        return selectedLEDColors;
    }

    public enum LEDColors {
        OFF(0, 0, 0, LEDMode.SOLID), RED(255, 0, 0, LEDMode.SOLID), GREEN(0, 255, 0, LEDMode.SOLID),
        BLUE(0, 0, 255, LEDMode.SOLID),
        DISABLED(255, 0, 0, LEDMode.SOLID), ENABLED(0, 0, 255, LEDMode.SOLID), ORANGE(255, 102, 0, LEDMode.SOLID),
        YELLOW(255, 100, 0, LEDMode.SOLID),
        PURPLE(255, 0, 255, LEDMode.SOLID),
        RAINBOW(0, 0, 0, LEDMode.RAINBOW), FIRE(0, 0, 0, LEDMode.FIRE), TWINKLE(0, 0, 0, LEDMode.TWINKLE),
        STROBE(0, 0, 0, LEDMode.STROBE),
        CONE(255, 100, 0, LEDMode.BLINK_SOLID), CUBE(255, 0, 255, LEDMode.BLINK_SOLID),
        REDFIRE(255, 0, 0, LEDMode.FIRE);

        int r;
        int g;
        int b;
        LEDMode ledMode;

        LEDColors(int r, int g, int b, LEDMode ledMode) {
            this.r = r;
            this.g = g;
            this.b = b;
            this.ledMode = ledMode;
        }
    }

    public LEDColors disabledLEDColorsMode = LEDColors.RAINBOW;
    private LEDColors currentLEDMode = LEDColors.OFF;

    public LEDColors getLEDMode() {
        return currentLEDMode;
    }

    public void configLEDs(LEDColors ledColors) {
        this.mRed = ledColors.r;
        this.mGreen = ledColors.g;
        this.mBlue = ledColors.b;
        this.selectedLEDType = ledColors.ledMode;
        currentLEDMode = ledColors;
        if(ledColors.ledMode == LEDMode.BLINK_SOLID) {
            blinkStopwatch.start();
        }
    }

    public void setBlinkMode(LEDBlink ledblink) {
        
    }

    public void setLEDsBlink(LEDColors ledColors) {
        // this.
    }

    private final Stopwatch blinkStopwatch = new Stopwatch();
    private static final double kBlinkRateSeconds = 0.25;

    @Override
    public void writePeriodicOutputs() {
        if (selectedLEDType == LEDMode.SOLID) {
            candle.setLEDs(mRed, mGreen, mBlue);
        } else if (selectedLEDType == LEDMode.RAINBOW) {
            RainbowAnimation animation = new RainbowAnimation(0.25, 0.25, 400);
            candle.animate(animation);
        } else if (selectedLEDType == LEDMode.FIRE) {
            FireAnimation fireAnimation = new FireAnimation(1, 1, 1690, 1, 0.25);
            candle.animate(fireAnimation);
        } else if (selectedLEDType == LEDMode.TWINKLE) {
            TwinkleAnimation twinkleAnimation = new TwinkleAnimation(255, 255, 255, 127, 0.25, 400,
                    TwinklePercent.Percent76);
            candle.animate(twinkleAnimation);
        } else if (selectedLEDType == LEDMode.STROBE) {
            candle.animate(new StrobeAnimation(100, 100, 100, 50, 0.25, 400));
        } else if (selectedLEDType == LEDMode.BLINK_SOLID) {
            // candle.animate()
            double time = blinkStopwatch.getTime();
            if (time < kBlinkRateSeconds / 2.0) {
                candle.setLEDs(mRed, mGreen, mBlue);
            } else if ((kBlinkRateSeconds / 2.0) < time && time < kBlinkRateSeconds) {
                candle.setLEDs(0, 0, 0);
            } else if (time > kBlinkRateSeconds) {
                blinkStopwatch.start();
            }
        }

    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("LEDs Mode", getLEDType().toString());
    }

    public Request ledModeRequest(LEDColors desiredColor) {
        return new Request() {
            @Override
            public void act() {
                configLEDs(desiredColor);
            }

        };
    }

    @Override
    public void stop() {
        configLEDs(disabledLEDColorsMode);
    }

    private LEDBlink currentBlinkProfile = new LEDBlink();
    public void setBlinkProfile(LEDBlink blinkProfile) {
        currentBlinkProfile = blinkProfile;
    }
    public LEDBlink getBlinkProfile() {
        return currentBlinkProfile;
    }
    private class LEDBlink {
        double onPeriod = 1.0;
        double offPeriod = 1.0;

        private Stopwatch stopwatch = new Stopwatch();

        public LEDBlink(double onPeriod, double offPeriod) {
            this.onPeriod = onPeriod;
            this.offPeriod = offPeriod;
        }

        public LEDBlink() {

        }

    }
}
