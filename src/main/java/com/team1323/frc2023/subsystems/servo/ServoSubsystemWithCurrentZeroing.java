package com.team1323.frc2023.subsystems.servo;

import java.util.List;

import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.lib.drivers.MotorController;

public abstract class ServoSubsystemWithCurrentZeroing<Inputs extends ServoSubsystemWithCurrentZeroingInputs> extends ServoSubsystem<Inputs> {
    private final CurrentZeroingConfig zeroingConfig;
    protected boolean isZeroed = false;

    public ServoSubsystemWithCurrentZeroing(MotorController leader, List<MotorController> followers, 
            ServoSubsystemConfig servoConfig, CurrentZeroingConfig zeroingConfig, Inputs inputs) {
        super(leader, followers, servoConfig, inputs);
        this.zeroingConfig = zeroingConfig;
    }

    @Override
    protected void updateInputsFromIO() {
        super.updateInputsFromIO();
        inputs.supplyCurrent = leader.getSupplyAmps();
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
            if (!isZeroed && leader.getSupplyAmps() > zeroingConfig.triggerSupplyCurrent) {
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
