package com.team254.drivers;

import com.ctre.phoenixpro.controls.ControlRequest;
import com.ctre.phoenixpro.hardware.TalonFX;

public class LazyPhoenixProTalonFX extends TalonFX {
    private static final String kDefaultCanBus = "rio";

    protected ControlRequest mLastControlRequest = null;

    public LazyPhoenixProTalonFX(int deviceNumber) {
        this(deviceNumber, kDefaultCanBus);
    }

    public LazyPhoenixProTalonFX(int deviceNumber, String canbus) {
        super(deviceNumber, kDefaultCanBus);
    }

    public void lazySetControl(ControlRequest controlRequest) {
        if (!controlRequest.equals(mLastControlRequest)) {
            setControl(controlRequest);
        }
    }
}
