package com.team1323.frc2023.auto.modes;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.team1323.frc2023.auto.AutoModeBase;
import com.team1323.frc2023.auto.AutoModeEndedException;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator.TrajectorySet.MirroredTrajectory;
import com.team254.lib.trajectory.timing.TimedState;

public class TestMode extends AutoModeBase {
    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths() {
        return Arrays.asList(trajectories.thirdPieceToSecondConeColumn.get(Quadrant.BOTTOM_LEFT), trajectories.thirdPieceToSecondConeColumn.get(Quadrant.TOP_LEFT),
                trajectories.thirdPieceToSecondConeColumn.get(Quadrant.TOP_RIGHT), trajectories.thirdPieceToSecondConeColumn.get(Quadrant.BOTTOM_RIGHT));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        
    }
}
