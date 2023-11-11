package com.team1323.frc2023.subsystems.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.subsystems.Subsystem;
import com.team1323.frc2023.subsystems.encoders.AbsoluteEncoder;
import com.team1323.frc2023.subsystems.encoders.MagEncoder;
import com.team1323.lib.math.Units;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.wpilib.SwerveModulePosition;

import edu.wpi.first.wpilibj.DriverStation;

public abstract class SwerveModule extends Subsystem {
	protected final AbsoluteEncoder rotationAbsoluteEncoder;
	protected final int moduleId;
	protected final String name;
	protected final double encoderOffsetDegrees;
	private boolean rotationMotorZeroed = false;
	private int zeroCount = 0;

	public SwerveModule(int moduleId, double encoderOffsetDegrees, boolean flipAbsoluteEncoder) {
		this.moduleId = moduleId;
		name = String.format("Module %d", moduleId);

		rotationAbsoluteEncoder = MagEncoder.createRealOrSimulatedEncoder(Ports.kModuleEncoders[moduleId], flipAbsoluteEncoder);
		this.encoderOffsetDegrees = encoderOffsetDegrees;
	}

	// Should be part of each module implementation's loggable inputs
	protected abstract double getAbsoluteEncoderDegrees();

	public abstract Rotation2d getAngle();

	public abstract boolean isAngleOnTarget();

	protected abstract ErrorCode setRotationSensorDegrees(double sensorDegrees);
	
	public abstract double getDriveDistanceInches();

	public abstract double getDriveVelocityInchesPerSecond();

	public abstract double getDriveVelocitySetpoint();

	public abstract double getDriveVoltage();

	public abstract boolean isDrivePositionOnTarget();
	
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(Units.inchesToMeters(getDriveDistanceInches()), getAngle());
	}

	public abstract void setDriveNeutralMode(NeutralMode mode);

	public abstract void setRotationNeutralMode(NeutralMode mode);
	
	/**
	 * @param driveVector A vector whose magnitude represents an open-loop
	 * drive speed in the range of [0, 1].
	 */
	public abstract void setOpenLoop(Translation2d driveVector);

	/**
	 * Causes the module to coast in the specified direction.
	 */
	public abstract void setOpenLoopCoast(Rotation2d driveDirection);

	/**
	 * @param driveVector A vector whose magnitude represents a drive velocity
	 * in inches per second.
	 */
	public abstract void setClosedLoopVelocity(Translation2d driveVector);

	/**
	 * Causes the module to point in the given direction and maintain a
	 * closed-loop velocity of 0.
	 */
	public abstract void setClosedLoopVelocityStall(Rotation2d driveDirection);

	/**
	 * @param driveVector A vector whose magnitude represents a drive distance
	 * in inches.
	 */
	public abstract void setClosedLoopPosition(Translation2d driveVector);

	/**
	 * Turns off all motors for this module. 
	 * 
	 * This is in contrast to the stop() method, which may keep some motors
	 * in closed-loop mode to maintain module heading.
	 */
	public abstract void disable();
	
	public void resetRotationToAbsolute() {
		if (!rotationMotorZeroed) {
			forceResetRotationToAbsolute();
		} 
	}

	public void forceResetRotationToAbsolute() {
		if (rotationAbsoluteEncoder.isConnected()) {
			setRotationSensorDegrees(getAbsoluteEncoderDegrees() - encoderOffsetDegrees);
		} else {
			setRotationSensorDegrees(0.0);
			DriverStation.reportError(String.format("MAG ENCODER FOR %s WAS NOT CONNECTED UPON BOOT!", name), false);
		}

		if (zeroCount < 500) {
			zeroCount++;
		} else {
			System.out.println(String.format("%s ZEROED.", name));
			rotationMotorZeroed = true;
		}
	}

	public void setRotationMotorZeroed(boolean isZeroed) {
		rotationMotorZeroed = isZeroed;
	}

	protected String getLogKey(String entryName) {
		return String.format("%s/%s", name, entryName);
	}

	public static class SwerveMotorInfo {
		public final int deviceId;
		public final TalonFXInvertType invertType;

		public SwerveMotorInfo(int deviceId, TalonFXInvertType invertType) {
			this.deviceId = deviceId;
			this.invertType = invertType;
		}
	}
}
