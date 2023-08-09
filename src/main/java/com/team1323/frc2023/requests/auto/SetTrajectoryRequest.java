package com.team1323.frc2023.requests.auto;

import com.team1323.frc2023.field.AutoZones;
import com.team1323.frc2023.field.AutoZones.Quadrant;
import com.team1323.frc2023.requests.Request;
import com.team1323.frc2023.subsystems.swerve.Swerve;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator.TrajectorySet.MirroredTrajectory;
import com.team254.lib.trajectory.timing.TimedState;

public class SetTrajectoryRequest extends Request {
	private final Trajectory<TimedState<Pose2dWithCurvature>> trajectory;
    private final Rotation2d goalHeading;
    private final double rotationScalar;
	private final Swerve swerve;

	public SetTrajectoryRequest(MirroredTrajectory trajectory, Rotation2d bottomLeftGoalHeading, double rotationScalar, Quadrant quadrant) {
		this(trajectory.get(quadrant), AutoZones.mirror(bottomLeftGoalHeading, quadrant), rotationScalar);
	}
	
	public SetTrajectoryRequest(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, Rotation2d goalHeading, double rotationScalar){
		this.trajectory = trajectory;
        this.goalHeading = goalHeading;
        this.rotationScalar = rotationScalar;
		swerve = Swerve.getInstance();
	}
	
	@Override
	public void act() {
		swerve.setTrajectory(trajectory, goalHeading, rotationScalar);
	}
}
