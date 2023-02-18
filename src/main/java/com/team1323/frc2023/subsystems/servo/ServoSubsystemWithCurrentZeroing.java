package com.team1323.frc2023.subsystems.servo;

import java.util.ArrayList;
import java.util.List;

import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;

public abstract class ServoSubsystemWithCurrentZeroing extends ServoSubsystem {
    private final CurrentZeroingConfig zeroingConfig;
    protected boolean isZeroed = false;

    public ServoSubsystemWithCurrentZeroing(int portNumber, String canBus, double encoderUnitsPerOutputUnit, 
            double minOutputUnits, double maxOutputUnits, double outputUnitTolerance, 
            double cruiseVelocityScalar, double accelerationScalar, CurrentZeroingConfig zeroingConfig) {
        this(portNumber, new ArrayList<>(), canBus, encoderUnitsPerOutputUnit, minOutputUnits, maxOutputUnits, 
                outputUnitTolerance, cruiseVelocityScalar, accelerationScalar, zeroingConfig);
    }

    public ServoSubsystemWithCurrentZeroing(int portNumber, List<Integer> followerPortNumbers, String canBus, 
            double encoderUnitsPerOutputUnit, double minOutputUnits, double maxOutputUnits, double outputUnitTolerance, 
            double cruiseVelocityScalar, double accelerationScalar, CurrentZeroingConfig zeroingConfig) {
        super(portNumber, followerPortNumbers, canBus, encoderUnitsPerOutputUnit, minOutputUnits, maxOutputUnits, 
                outputUnitTolerance, cruiseVelocityScalar, accelerationScalar);
        this.zeroingConfig = zeroingConfig;
    }

    @Override
    protected void zeroPosition() {
        leader.setSelectedSensorPosition(outputUnitsToEncoderUnits(zeroingConfig.homingOutputUnits));
    }

    public void startCurrentZeroing() {
        isZeroed = false;
        enableLimits(false);
        super.setOpenLoop(zeroingConfig.outputPercent);
    }

    private Loop zeroingLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            if (!isZeroed) {
                startCurrentZeroing();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            if (!isZeroed && leader.getSupplyCurrent() > zeroingConfig.triggerSupplyCurrent) {
                zeroPosition();
                enableLimits(true);
                isZeroed = true;
                setPosition(zeroingConfig.targetOutputUnitsAfterZeroing);
            }
        }

        @Override
        public void onStop(double timestamp) {}
    };

    // If a subclass overrides this method, it should make sure to call
    // super.registerEnabledLoops() to ensure that the zeroing behavior works.
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(zeroingLoop);
    }

    @Override
    public void setPosition(double outputUnits) {
        if (isZeroed) {
            super.setPosition(outputUnits);
        }
    }

    @Override
    public void lockPosition() {
        if (isZeroed) {
            super.lockPosition();
        }
    }

    @Override
    protected void setOpenLoop(double percent) {
        if (isZeroed) {
            super.setOpenLoop(percent);
        }
    }

    public static class CurrentZeroingConfig {
        public final double outputPercent;
        public final double triggerSupplyCurrent;
        public final double homingOutputUnits;
        public final double targetOutputUnitsAfterZeroing;

        public CurrentZeroingConfig(double outputPercent, double triggerSupplyCurrent, 
                double homingOutputUnits, double targetOutputUnitsAfterZeroing) {
            this.outputPercent = outputPercent;
            this.triggerSupplyCurrent = triggerSupplyCurrent;
            this.homingOutputUnits = homingOutputUnits;
            this.targetOutputUnitsAfterZeroing = targetOutputUnitsAfterZeroing;
        }
    }
}
