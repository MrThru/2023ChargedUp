package com.team1323.frc2023.subsystems.swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.lib.drivers.MotorController;
import com.team1323.lib.drivers.Phoenix5FXMotorController;
import com.team1323.lib.drivers.Phoenix6FXMotorController;
import com.team1323.lib.util.LogUtil;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class CoaxialSwerveModule extends SwerveModule {
    private final MotorController rotationMotor, leadDriveMotor;
    private final List<MotorController> allDriveMotors, followerDriveMotors;
    private final CoaxialSwerveInputsAutoLogged inputs = new CoaxialSwerveInputsAutoLogged();
    private final CoaxialSwerveOutputs outputs = new CoaxialSwerveOutputs();

    public CoaxialSwerveModule(int moduleId, double encoderOffsetDegrees, boolean flipAbsoluteEncoder,
            SwerveMotorInfo rotationMotorInfo, SwerveMotorInfo leadDriveMotorInfo, SwerveMotorInfo... followerDriveMotorInfos) {
        super(moduleId, encoderOffsetDegrees, flipAbsoluteEncoder);

        rotationMotor = Phoenix5FXMotorController.createRealOrSimulatedController(rotationMotorInfo.deviceId, Ports.CANBUS);
        rotationMotor.configureAsCoaxialSwerveRotation();
        rotationMotor.setInverted(rotationMotorInfo.invertType);

        leadDriveMotor = createDriveMotor(leadDriveMotorInfo);
        followerDriveMotors = Arrays.stream(followerDriveMotorInfos)
                .map(info -> createDriveMotor(info))
                .toList();
        allDriveMotors = new ArrayList<>();
        allDriveMotors.add(leadDriveMotor);
        allDriveMotors.addAll(followerDriveMotors);

        followerDriveMotors.forEach(m -> m.set(ControlMode.Follower, leadDriveMotorInfo.deviceId));
    }

    private MotorController createDriveMotor(SwerveMotorInfo motorInfo) {
        MotorController driveMotor = Phoenix6FXMotorController.createRealOrSimulatedController(motorInfo.deviceId, Ports.CANBUS, true);
        driveMotor.configureAsCoaxialSwerveDrive();
        driveMotor.setInverted(motorInfo.invertType);

        return driveMotor;
    }

    @Override
    protected double getAbsoluteEncoderDegrees() {
        return inputs.absoluteRotationDegrees;
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(getUnboundedAngle());
    }

    @Override
    public boolean isAngleOnTarget() {
        if (outputs.rotationControlMode == ControlMode.MotionMagic) {
            double errorDegrees = encUnitsToDegrees(Math.abs(outputs.rotationDemand - inputs.rotationPosition));
            return errorDegrees < Constants.kSwerveModuleRotationTolerance;
        }

        return false;
    }

    @Override
    protected ErrorCode setRotationSensorDegrees(double sensorDegrees) {
        return rotationMotor.setSelectedSensorPosition(degreesToEncUnits(sensorDegrees));
    }

    @Override
    public double getDriveDistanceInches() {
        return encUnitsToInches(inputs.drivePosition);
    }

    @Override
    public double getDriveVelocityInchesPerSecond() {
        return encVelocityToInchesPerSecond(inputs.driveVelocity);
    }

    @Override
    public double getDriveVelocitySetpoint() {
		if (outputs.driveControlMode == ControlMode.Velocity) {
            return outputs.driveDemand;
		}

        return 0.0;
    }

    @Override
    public double getDriveVoltage() {
        return leadDriveMotor.getAppliedVoltage();
    }

    @Override
    public boolean isDrivePositionOnTarget() {
        if (outputs.driveControlMode == ControlMode.MotionMagic) {
            return encUnitsToInches(Math.abs(outputs.driveDemand - inputs.drivePosition)) < Constants.kSwerveMotionMagicTolerance;
        }

        return false;
    }

    @Override
    public void setNeutralMode(NeutralMode mode) {
        rotationMotor.setNeutralMode(mode);
        allDriveMotors.forEach(m -> m.setNeutralMode(mode));
    }

    @Override
    public void setOpenLoop(Translation2d driveVector) {
        Rotation2d driveDirection = driveVector.direction();

        if (Util.shouldReverse(driveDirection, getAngle())) {
            setAngle(driveDirection.rotateBy(Rotation2d.fromDegrees(180.0)));
            outputs.driveDemand = -driveVector.norm();
        } else {
            setAngle(driveDirection);
            outputs.driveDemand = driveVector.norm();
        }

        outputs.driveControlMode = ControlMode.PercentOutput;
    }

    @Override
    public void setOpenLoopCoast(Rotation2d driveDirection) {
        if (Util.shouldReverse(driveDirection, getAngle())) {
            setAngle(driveDirection.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else {
            setAngle(driveDirection);
        }

        outputs.driveControlMode = ControlMode.PercentOutput;
        outputs.driveDemand = 0.0;
    }

    @Override
    public void setClosedLoopVelocity(Translation2d driveVector) {
        Rotation2d driveDirection = driveVector.direction();

        if (Util.shouldReverse(driveDirection, getAngle())) {
            setAngle(driveDirection.rotateBy(Rotation2d.fromDegrees(180.0)));
            outputs.driveDemand = -inchesPerSecondToEncVelocity(driveVector.norm());
        } else {
            setAngle(driveDirection);
            outputs.driveDemand = inchesPerSecondToEncVelocity(driveVector.norm());
        }

        leadDriveMotor.selectProfileSlot(1);
        outputs.driveControlMode = ControlMode.Velocity;
    }

    @Override
    public void setClosedLoopVelocityStall(Rotation2d driveDirection) {
        if (Util.shouldReverse(driveDirection, getAngle())) {
            setAngle(driveDirection.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else {
            setAngle(driveDirection);
        }

        outputs.driveControlMode = ControlMode.Velocity;
        outputs.driveDemand = 0.0;
    }

    @Override
    public void setClosedLoopPosition(Translation2d driveVector) {
        setAngle(driveVector.direction());

        leadDriveMotor.selectProfileSlot(0);
		outputs.driveControlMode = ControlMode.MotionMagic;
		outputs.driveDemand = inputs.drivePosition + inchesToEncUnits(driveVector.norm());
    }

    @Override
    public void stop() {
        setAngle(getAngle());

        outputs.driveControlMode = ControlMode.PercentOutput;
        outputs.driveDemand = 0.0;
    }

    @Override
    public void disable() {
        outputs.rotationControlMode = ControlMode.PercentOutput;
        outputs.rotationDemand = 0.0;

        outputs.driveControlMode = ControlMode.PercentOutput;
        outputs.driveDemand = 0.0;
    }

	private double encUnitsToInches(double encUnits) {
		return encUnits / Constants.kSwerveEncUnitsPerInch;
	}
	
	private double inchesToEncUnits(double inches) {
		return inches * Constants.kSwerveEncUnitsPerInch;
	}
	
	private double encVelocityToInchesPerSecond(double encUnitsPer100ms) {
		return encUnitsToInches(encUnitsPer100ms) * 10;
	}
	
	private double inchesPerSecondToEncVelocity(double inchesPerSecond) {
		return inchesToEncUnits(inchesPerSecond / 10.0);
	}
	
	private double degreesToEncUnits(double degrees) {
		return (degrees / 360.0) * Constants.kSwerveRotationReduction * Constants.kSwerveRotationEncoderResolution;
	}
	
	private double encUnitsToDegrees(double encUnits) {
		return (encUnits / Constants.kSwerveRotationEncoderResolution) / Constants.kSwerveRotationReduction * 360.0;
	}
	
	private double getUnboundedAngle() {
		return encUnitsToDegrees(inputs.rotationPosition);
	}

	private void setAngle(Rotation2d goalAngle){
		double newAngle = Util.placeInAppropriate0To360Scope(getUnboundedAngle(), goalAngle.getDegrees());
		double setpoint = degreesToEncUnits(newAngle);
		outputs.rotationControlMode = ControlMode.MotionMagic;
		outputs.rotationDemand = setpoint;
	}
	
    @Override
    public void readPeriodicInputs() {
        inputs.rotationPosition = rotationMotor.getSelectedSensorPosition();
        inputs.drivePosition = leadDriveMotor.getSelectedSensorPosition();
        inputs.driveVelocity = leadDriveMotor.getVelocityEncoderUnitsPer100Ms();
        inputs.absoluteRotationDegrees = rotationAbsoluteEncoder.getDegrees();
        Logger.getInstance().processInputs(name, inputs);
    }

    @Override
    public void writePeriodicOutputs() {
        rotationMotor.set(outputs.rotationControlMode, outputs.rotationDemand);
        leadDriveMotor.set(outputs.driveControlMode, outputs.driveDemand);
    }

    @Override
    public void outputTelemetry() {
        LogUtil.recordRotation2d(getLogKey("Angle"), getAngle());
        Logger.getInstance().recordOutput(getLogKey("Inches Driven"), getDriveDistanceInches());
        Logger.getInstance().recordOutput(getLogKey("Inches Per Second"), getDriveVelocityInchesPerSecond());
    }

    @AutoLog
    public static class CoaxialSwerveInputs {
        public double rotationPosition;
        public double drivePosition;
        public double driveVelocity;
        public double absoluteRotationDegrees;
    }

    public static class CoaxialSwerveOutputs {
        public ControlMode rotationControlMode = ControlMode.PercentOutput;
        public double rotationDemand = 0.0;
        public ControlMode driveControlMode = ControlMode.PercentOutput;
        public double driveDemand = 0.0;
    }
}
