package com.team1323.frc2023.requests.auto.routines;

import com.team1323.frc2023.requests.Request;
import com.team1323.lib.util.Stopwatch;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.TrajectoryGenerator.TrajectorySet;

public abstract class AutoRoutine {
    protected final TrajectorySet trajectories;
    protected final Stopwatch runtimeStopwatch = new Stopwatch();

    public AutoRoutine() {
        trajectories = TrajectoryGenerator.getInstance().getTrajectorySet();
    }

    public abstract Request getRoutine();
}
