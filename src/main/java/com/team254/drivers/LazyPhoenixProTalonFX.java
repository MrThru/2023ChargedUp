package com.team254.drivers;

import com.ctre.phoenixpro.controls.ControlRequest;
import com.ctre.phoenixpro.hardware.TalonFX;

public class LazyPhoenixProTalonFX extends TalonFX {
    private static final String kDefaultCanBus = "rio";

    private ControlRequest mLastRequest = null;

    public LazyPhoenixProTalonFX(int deviceNumber) {
        this(deviceNumber, kDefaultCanBus);
    }

    public LazyPhoenixProTalonFX(int deviceNumber, String canbus) {
        super(deviceNumber, canbus);
    }

    public void set(ControlRequest controlRequest) {
        if (!controlRequest.equals(mLastRequest)) {
            mLastRequest = controlRequest;
            super.setControl(controlRequest);
        }
    }
}
