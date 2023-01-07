/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2023;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.team1323.frc2023.vision.GoalTracker;
import com.team1323.frc2023.vision.GoalTracker.TrackReport;
import com.team1323.frc2023.vision.ShooterAimingParameters;
import com.team1323.frc2023.vision.TargetInfo;
import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.InterpolatingTreeMap;
import com.team1323.lib.util.Kinematics;
// Dope poof libs that take care of robot motion
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotState {
	
	private static RobotState instance = new RobotState();
	public static RobotState getInstance(){
		return instance;
	}
	
	private static final int kObservationBufferSize = 100;
	
	public static final Pose2d kVehicleToTurretFixed = Pose2d.identity();
	
	public static final Pose2d kTurretRotatingToCamera = new Pose2d(
			new Translation2d(Constants.kCameraXOffset, Constants.kCameraYOffset), new Rotation2d());
		
	private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
	private InterpolatingTreeMap<InterpolatingDouble, Rotation2d> turret_rotation_;
	private GoalTracker goal_tracker_;
	private Rotation2d camera_pitch_correction_;
	private Rotation2d camera_yaw_correction_;
	private double differential_height_;
	private double distance_driven_;
	private Twist2d vehicle_velocity_;
	private ShooterAimingParameters cached_shooter_aiming_params_ = null;
	
	private Translation2d lastKnownTargetPosition = new Translation2d();
	public Translation2d lastKnownTargetPosition(){ return lastKnownTargetPosition; }
	
	public double distanceToTarget(){
		return getLatestFieldToVehicle().getValue().getTranslation().distance(lastKnownTargetPosition);
	}
	
	private boolean seesTarget = false;
	public boolean seesTarget(){
		return seesTarget;
	}
	
	private RobotState() {
		reset(0, new Pose2d(), new Rotation2d());
	}
	
	/**
	* Resets the field to robot transform (robot's position on the field)
	*/
	public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle,
			Rotation2d initial_turret_rotation) {
		field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
		field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
		turret_rotation_ = new InterpolatingTreeMap<>(kObservationBufferSize);
		turret_rotation_.put(new InterpolatingDouble(start_time), initial_turret_rotation);
		goal_tracker_ = new GoalTracker();
		camera_pitch_correction_ = Rotation2d.fromDegrees(-Constants.kCameraPitchAngleDegrees);
		camera_yaw_correction_ = Rotation2d.fromDegrees(-Constants.kCameraYawAngleDegrees);
		differential_height_ = Constants.kVisionTargetHeight - Constants.kCameraZOffset;
		distance_driven_ = 0.0;
		vehicle_velocity_ = Twist2d.identity();
	}
	
	public synchronized void resetDistanceDriven() {
		distance_driven_ = 0.0;
	}
	
	/**
	* Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
	* to fill in the gaps.
	*/
	public synchronized Pose2d getFieldToVehicle(double timestamp) {
		return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
	}
	
	public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
		return field_to_vehicle_.lastEntry();
	}

	public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
		return getLatestFieldToVehicle().getValue()
				.transformBy(Pose2d.exp(vehicle_velocity_.scaled(lookahead_time)));
	}
	
	public synchronized Rotation2d getTurretRotation(double timestamp) {
		return turret_rotation_.getInterpolated(new InterpolatingDouble(timestamp));
	}
	
	public synchronized Map.Entry<InterpolatingDouble, Rotation2d> getLatestTurretRotation() {
		return turret_rotation_.lastEntry();
	}
	
	public synchronized Pose2d getFieldToTurretRotated(double timestamp) {
		InterpolatingDouble key = new InterpolatingDouble(timestamp);
		return field_to_vehicle_.getInterpolated(key).transformBy(kVehicleToTurretFixed)
				.transformBy(Pose2d.fromRotation(turret_rotation_.getInterpolated(key)));
	}
	
	public synchronized Pose2d getFieldToCamera(double timestamp) {
		return getFieldToTurretRotated(timestamp).transformBy(kTurretRotatingToCamera);
	}
	
	public void addVisionUpdate(double timestamp, List<TargetInfo> vision_update) {
		List<Translation2d> field_to_goals = new ArrayList<>();
		Pose2d field_to_camera = getFieldToCamera(timestamp);
		if (!(vision_update == null || vision_update.isEmpty())) {
			seesTarget = true;
			for (TargetInfo target : vision_update) {
				double ydeadband = target.getY();
				
				// Compensate for camera yaw
				double xyaw = target.getX() * camera_yaw_correction_.cos() + ydeadband * camera_yaw_correction_.sin();
				double yyaw = ydeadband * camera_yaw_correction_.cos() - target.getX() * camera_yaw_correction_.sin();
				double zyaw = target.getZ();
				
				// Compensate for camera pitch
				double xr = zyaw * camera_pitch_correction_.sin() + xyaw * camera_pitch_correction_.cos();
				double yr = yyaw;
				double zr = zyaw * camera_pitch_correction_.cos() - xyaw * camera_pitch_correction_.sin();
				
				// find intersection with the goal
				//if (zr < 0) {
					double scaling = differential_height_ / zr;
					double distance = (Math.hypot(xr, yr) * scaling) + Constants.kVisionTargetRadius;
					Rotation2d angle = new Rotation2d(xr, yr, true);
					field_to_goals.add(field_to_camera
					.transformBy(Pose2d
					.fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin())))
					.getTranslation());
					//}
				}
			}else{
				seesTarget = false;
			}
			synchronized (this) {
				goal_tracker_.update(timestamp, field_to_goals);
			}
		}
		
		public synchronized Optional<ShooterAimingParameters> getCachedAimingParameters() {
			return cached_shooter_aiming_params_ == null ? Optional.empty() : Optional.of(cached_shooter_aiming_params_);
		}

		public synchronized Optional<ShooterAimingParameters> getAimingParameters() {
			return Optional.empty();
		}

		public synchronized Optional<Pose2d> getEstimatedRobotPosition() {
			List<TrackReport> reports = goal_tracker_.getTracks();
			if (!reports.isEmpty()) {
				TrackReport report = reports.get(0);
				Translation2d latest_field_to_vehicle = getLatestFieldToVehicle().getValue().getTranslation();
				Translation2d latest_vehicle_to_goal = new Translation2d(latest_field_to_vehicle, report.field_to_goal);
				Translation2d calculated_center_of_field = (new Pose2d(report.field_to_goal, latest_vehicle_to_goal.direction()))
						.transformBy(Pose2d.fromTranslation(new Translation2d(/*Constants.kVisionTargetRadius*/0.0, 0.0))).getTranslation();
				Translation2d center_of_field_to_vehicle = new Translation2d(calculated_center_of_field, latest_field_to_vehicle);
				Translation2d adjusted_field_to_vehicle = Constants.kCenterOfField.translateBy(center_of_field_to_vehicle);

				return Optional.of(new Pose2d(adjusted_field_to_vehicle, getLatestFieldToVehicle().getValue().getRotation()));
			} else {
				return Optional.empty();
			}
		}

		public synchronized Translation2d getTurretToCenterOfField() {
			Pose2d latest_turret_fixed_to_field = getPredictedFieldToVehicle(Constants.kAutoAimPredictionTime)
						.transformBy(kVehicleToTurretFixed).inverse();
			Pose2d latest_turret_fixed_to_cof = latest_turret_fixed_to_field
						.transformBy(Pose2d.fromTranslation(Constants.kCenterOfField));
			
			return latest_turret_fixed_to_cof.getTranslation();
		}
		
		public synchronized Optional<Pose2d> getRobotScoringPosition(Optional<ShooterAimingParameters> aimingParameters, Rotation2d orientation, Translation2d endTranslation){
			List<Pose2d> targetPositions = getCaptureTimeFieldToGoal();
			if(!targetPositions.isEmpty() && aimingParameters.isPresent()){
				Translation2d targetPosition = targetPositions.get(0).getTranslation();
				SmartDashboard.putNumberArray("Path Pose", new double[]{targetPosition.x(), targetPosition.y(), 0.0, 0.0}); 
				Pose2d orientedTargetPosition = new Pose2d(targetPosition, orientation).transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
				Pose2d robotScoringPosition = orientedTargetPosition.transformBy(Pose2d.fromTranslation(endTranslation));
				
				return Optional.of(robotScoringPosition);
			}
			return Optional.empty();
		}
		
		public synchronized List<Pose2d> getCaptureTimeFieldToGoal() {
			List<Pose2d> rv = new ArrayList<>();
			for (TrackReport report : goal_tracker_.getTracks()) {
				rv.add(Pose2d.fromTranslation(report.field_to_goal));
			}
			return rv;
		}
		
		public synchronized void clearVisionTargets(){
			goal_tracker_.clearTracks();
		}
		
		public synchronized void resetVision() {
			goal_tracker_.reset();
		}
		
		public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
			field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
		}
		
		public synchronized void addTurretRotationObservation(double timestamp, Rotation2d observation) {
			turret_rotation_.put(new InterpolatingDouble(timestamp), observation);
		}
		
		public synchronized double getDistanceDriven() {
			return distance_driven_;
		}

        public synchronized void addObservations(double timestamp, Pose2d pose, Twist2d velocity, Rotation2d turretAngle) {
            addFieldToVehicleObservation(timestamp, pose);
            addTurretRotationObservation(timestamp, turretAngle);
            vehicle_velocity_ = velocity;
            /*if (Math.abs(vehicle_velocity_.dtheta) < 2.0 * Math.PI) {
                vehicle_velocity_averaged_.add(vehicle_velocity_);
            } else {
                vehicle_velocity_averaged_.add(new Twist2d(vehicle_velocity_.dx, vehicle_velocity_.dy, 0.0));
            }*/
        }
		
		public synchronized void addObservations(double timestamp, Twist2d field_to_vehicle, Rotation2d turret_rotation) {
			addFieldToVehicleObservation(timestamp, Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), field_to_vehicle));
			addTurretRotationObservation(timestamp, turret_rotation);
			vehicle_velocity_ = field_to_vehicle;
		}
		
		public void outputToSmartDashboard(){
			SmartDashboard.putBoolean("Sees Target", seesTarget);
			SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
			if(true){
				List<Pose2d> poses = getCaptureTimeFieldToGoal();
				for (Pose2d pose : poses) {
					// Only output first goal
					SmartDashboard.putNumber("goal_pose_x", pose.getTranslation().x());
					SmartDashboard.putNumber("goal_pose_y", pose.getTranslation().y());
					
					break;
				}
				Optional<ShooterAimingParameters> aiming_params = getCachedAimingParameters();
				if (aiming_params.isPresent()) {
					SmartDashboard.putNumber("goal_range", aiming_params.get().getRange());
					SmartDashboard.putNumber("goal_theta", aiming_params.get().getTurretAngle().getDegrees());
				} else {
					SmartDashboard.putNumber("goal_range", 0.0);
					SmartDashboard.putNumber("goal_theta", 0.0);
				}
			}
		}
	}
	