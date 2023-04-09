package com.team1323.frc2023.auto.modes;

import java.util.Arrays;
import java.util.List;

import com.team1323.frc2023.auto.AutoModeBase;
import com.team1323.frc2023.auto.AutoModeEndedException;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

public class TestMode extends AutoModeBase {
    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.stream(Quadrant.values())
                .map(quadrant -> trajectories.secondPiecePickupPath.get(quadrant))
                .toList();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        
    }
}
