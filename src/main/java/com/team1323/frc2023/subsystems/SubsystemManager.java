package com.team1323.frc2023.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager extends Subsystem implements ILooper, Loop {
    private final List<Subsystem> mAllSubsystems;
    private final List<Loop> mLoops = new ArrayList<>();

    public SubsystemManager(List<Subsystem> allSubsystems) {
        mAllSubsystems = allSubsystems;
        mAllSubsystems.forEach(s -> s.registerEnabledLoops(this));
    }

    @Override
    public void readPeriodicInputs() {
        mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
    }

    @Override
    public void writePeriodicOutputs() {
        mAllSubsystems.forEach(Subsystem::writePeriodicOutputs);
    }

    @Override
    public void outputTelemetry() {
        mAllSubsystems.forEach(Subsystem::outputTelemetry);
    }

    @Override
    public void stop() {
        mAllSubsystems.forEach(Subsystem::stop);
    }

    @Override
    public void zeroSensors() {
        mAllSubsystems.forEach(Subsystem::zeroSensors);
    }

    public boolean hasEmergency(){
        boolean emergency = false;
        for(Subsystem s : mAllSubsystems){
            emergency |= s.hasEmergency;
        }
        return emergency;
    }

    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
    }

    @Override
    public void onStart(double timestamp) {
        mLoops.forEach(l -> l.onStart(timestamp));
    }

    @Override
    public void onLoop(double timestamp) {
        mLoops.forEach(l -> l.onLoop(timestamp));
    }

    @Override
    public void onStop(double timestamp) {
        mLoops.forEach(l -> l.onStop(timestamp));
    }
}
