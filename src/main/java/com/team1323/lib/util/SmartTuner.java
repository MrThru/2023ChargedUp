// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.lib.util;

import com.team1323.frc2023.Constants;
import com.team254.drivers.LazyPhoenix5TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SmartTuner {

    private LazyPhoenix5TalonFX motor;
    private String keyName;

    private boolean enabled = false;
    boolean hasBeenEnabled = false;

    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double mValue;
    public double getValue() {
        return mValue;
    }
    private MicroTuner[] mMicroTuners = new MicroTuner[0];



    public SmartTuner(LazyPhoenix5TalonFX motorInstance, String dashboardKeyName) {
        motor = motorInstance;
        keyName = dashboardKeyName;
    }
    public SmartTuner(String dashboardKeyName) {
        keyName = dashboardKeyName;
        createDashboardKey(keyName + "Input", 0);
    }
    public void initalizeWidgets() {
        createDashboardKey(keyName + "kP", 0);
        createDashboardKey(keyName + "kI", 0);
        createDashboardKey(keyName + "kD", 0);
        createDashboardKey(keyName + "kF", 0);
        createDashboardKey(keyName + "Input", 0);
    }

    public void update() {
        if(this.enabled) {
            updatekP(getDashboardNumberValue(keyName + "kP"));
            updatekI(getDashboardNumberValue(keyName + "kI"));
            updatekD(getDashboardNumberValue(keyName + "kD"));
            updatekF(getDashboardNumberValue(keyName + "kF"));
            mValue = getDashboardNumberValue(keyName + "Input");
            updateMicroTuners();
        }
    }
    
    
    public void createDashboardKey(String keyName, String value) {
        value = getDashboardStringValue(keyName);
        SmartDashboard.putString(keyName, value);
    }
    public void createDashboardKey(String keyName, double value) {
        value = getDashboardNumberValue(keyName);
        SmartDashboard.putNumber(keyName, value);
    }

    public double getDashboardNumberValue(String keyName) {
        return SmartDashboard.getNumber(keyName, 0);
    }
    public String getDashboardStringValue(String keyName) {
        return SmartDashboard.getString(keyName, "");
    }

    public MicroTuner linkMicroTuner(MicroTuner microTuner) {
        MicroTuner[] tempMicroTuners = new MicroTuner[mMicroTuners.length + 1];
        for(int i = 0; i < mMicroTuners.length; i++) {
            tempMicroTuners[i] = mMicroTuners[i];
        }
        tempMicroTuners[mMicroTuners.length] = microTuner;
        mMicroTuners = tempMicroTuners;
        return microTuner;
    }
    public void updateMicroTuners() {
        for(int i = 0; i < mMicroTuners.length; i++) {
            mMicroTuners[i].update();
        }
    }

    public void updatekP(double value) {
        if(kP != value) {
            kP = value;
            motor.config_kP(0, value, Constants.kCANTimeoutMs);
        }
    }
    public void updatekI(double value) {
        if(kI != value) {
            kI = value;
            motor.config_kI(0, value, Constants.kCANTimeoutMs);
        }
    }
    public void updatekD(double value) {
        if(kD != value) {
            kD = value;
            motor.config_kD(0, value, Constants.kCANTimeoutMs);
        }
    }
    public void updatekF(double value) {
        if(kF != value) {
            kF = value;
            motor.config_kF(0, value, Constants.kCANTimeoutMs);
        }
    }

    public void enabled(boolean enabled) {
        if(enabled && !hasBeenEnabled)
            hasBeenEnabled = true;
            initalizeWidgets();
        this.enabled = enabled;
    }

    public class MicroTuner {
        String keyname;
        char valueType;
        double valueNumber;
        String valueString;
        public MicroTuner(String keyname, char valueTypeChar) {
            this.keyname = keyname;
            this.valueType = valueTypeChar;
            if(valueTypeChar == 's') {
                createDashboardKey(keyName, "");
            } else if(valueTypeChar == 'n') {
                createDashboardKey(keyName, 0);
            }
        }
        public void update() {
            if(this.valueType == 's') {
                this.valueString = getDashboardStringValue(this.keyname);
            } else if(this.valueType == 'n') {
                this.valueNumber = getDashboardNumberValue(this.keyname);
            }
        }
        public Object getValue() {
            if(this.valueType == 's') {
                return this.valueString;
            } else {
                return this.valueNumber;
            }
        }
    }
}
