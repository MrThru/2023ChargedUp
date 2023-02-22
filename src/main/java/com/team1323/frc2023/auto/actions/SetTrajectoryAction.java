package com.team1323.frc2023.auto.actions;

import com.team1323.frc2023.field.AutoZones;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator.TrajectorySet.MirroredTrajectory;
import com.team254.lib.trajectory.timing.TimedState;

public class SetTrajectoryAction extends RunOnceAction{
	Trajectory<TimedState<Pose2dWithCurvature>> trajectory;
    Rotation2d goalHeading;
    double rotationScalar;
	Swerve swerve;

	public SetTrajectoryAction(MirroredTrajectory trajectory, Rotation2d bottomLeftGoalHeading, double rotationScalar, Quadrant quadrant) {
		this(trajectory.get(quadrant), AutoZones.mirror(bottomLeftGoalHeading, quadrant), rotationScalar);
	}
	
	public SetTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, Rotation2d goalHeading, double rotationScalar){
		this.trajectory = trajectory;
        this.goalHeading = goalHeading;
        this.rotationScalar = rotationScalar;
		swerve = Swerve.getInstance();
	}
	
	@Override
	public synchronized void runOnce(){
		swerve.setTrajectory(trajectory, goalHeading, rotationScalar);
	}
}
