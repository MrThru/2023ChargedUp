package com.team1323.frc2023.loops;

import java.util.ArrayList;
import java.util.List;

import com.team1323.frc2023.subsystems.SubsystemManager;

public class SynchronousLooper implements ILooper {
    private final SubsystemManager subsystems;
    private final List<Loop> autoLoops = new ArrayList<>();
    private final List<Loop> teleopLoops = new ArrayList<>();
    private final List<Loop> disabledLoops = new ArrayList<>();

    public SynchronousLooper(SubsystemManager subsystems) {
        this.subsystems = subsystems;
    }

    public void registerAutoLoop(Loop loop) {
        autoLoops.add(loop);
    }

    public void registerTeleopLoop(Loop loop) {
        teleopLoops.add(loop);
    }

    public void registerDisabledLoop(Loop loop) {
        disabledLoops.add(loop);
    }

    /**
     * Register a Loop that will run throughout all periods
     * (auto, teleop, and disabled).
     */
    @Override
    public void register(Loop loop) {
        registerAutoLoop(loop);
        registerTeleopLoop(loop);
        registerDisabledLoop(loop);
    }

    private void startEnabled(double timestamp, List<Loop> auxiliaryLoops) {
        disabledLoops.forEach(l -> l.onStop(timestamp));
        subsystems.onStart(timestamp);
        auxiliaryLoops.forEach(l -> l.onStart(timestamp));
    }

    public void startAuto(double timestamp) {
        startEnabled(timestamp, autoLoops);
    }

    public void startTeleop(double timestamp) {
        startEnabled(timestamp, teleopLoops);
    }

    public void startDisabled(double timestamp) {
        autoLoops.forEach(l -> l.onStop(timestamp));
        teleopLoops.forEach(l -> l.onStop(timestamp));
        subsystems.onStop(timestamp);
        subsystems.stop();
        disabledLoops.forEach(l -> l.onStart(timestamp));
    }

    private void onEnabledLoop(double timestamp, List<Loop> auxiliaryLoops) {
        subsystems.readPeriodicInputs();
        auxiliaryLoops.forEach(l -> l.onLoop(timestamp));
        subsystems.onLoop(timestamp);
        subsystems.writePeriodicOutputs();
        subsystems.outputTelemetry();
    }

    public void onAutoLoop(double timestamp) {
        onEnabledLoop(timestamp, autoLoops);
    }

    public void onTeleopLoop(double timestamp) {
        onEnabledLoop(timestamp, teleopLoops);
    }

    public void onDisabledLoop(double timestamp) {
        subsystems.readPeriodicInputs();
        disabledLoops.forEach(l -> l.onLoop(timestamp));
        subsystems.writePeriodicOutputs();
        subsystems.outputTelemetry();
    }
}
