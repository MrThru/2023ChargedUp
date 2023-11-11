package com.team1323.frc2023.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.lib.drivers.MotorController;
import com.team1323.lib.drivers.Phoenix5FXMotorController;
import com.team1323.lib.util.SynchronousPIDF;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class DifferentialSwerveModule extends SwerveModule {
    private static final double kMaxOpenLoopDifferentialOutput = 0.4;
    private static final double kMaxOpenLoopDriveSetpoint = 0.5;
    private static final double kMaxOpenLoopMotorOutput = 0.5;

    private final MotorController leftMotor, rightMotor;

    private final DifferentialSwerveInputsAutoLogged inputs = new DifferentialSwerveInputsAutoLogged();
    private final DifferentialSwerveOutputs outputs = new DifferentialSwerveOutputs();

    private final SynchronousPIDF openLoopRotationPID = new SynchronousPIDF(0.01, 0.0, 0.0, 0.0);

    /**
     * The motors should be inverted such that positive output to the left motor rotates the module clockwise
     * (from a top-down view) and positive output to the right motor rotates the module counter-clockwise.
     */
    public DifferentialSwerveModule(int moduleId, double encoderOffsetDegrees, boolean flipAbsoluteEncoder,
            SwerveMotorInfo leftMotorInfo, SwerveMotorInfo rightMotorInfo) {
        super(moduleId, encoderOffsetDegrees, flipAbsoluteEncoder);
        
        leftMotor = Phoenix5FXMotorController.createRealOrSimulatedController(leftMotorInfo.deviceId, Ports.CANBUS);
        leftMotor.configureAsDifferentialSwerveMotor();
        leftMotor.setInverted(leftMotorInfo.invertType);

        rightMotor = Phoenix5FXMotorController.createRealOrSimulatedController(rightMotorInfo.deviceId, Ports.CANBUS);
        rightMotor.configureAsDifferentialSwerveMotor();
        rightMotor.setInverted(rightMotorInfo.invertType);

        openLoopRotationPID.setSetpoint(0.0);

        // TODO: remove this
        leftMotor.setSelectedSensorPosition(0);
        rightMotor.setSelectedSensorPosition(0);
    }

    /**
     * Temporary method for initial testing. Should be removed later on.
     */
    public void setMotorOutputs(double leftMotorPercentOutput, double rightMotorPercentOutput) {
        outputs.controlMode = ControlMode.PercentOutput;
        outputs.leftMotorDemand = leftMotorPercentOutput;
        outputs.rightMotorDemand = rightMotorPercentOutput;
    }

    @Override
    protected double getAbsoluteEncoderDegrees() {
        return inputs.absoluteRotationDegrees;
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(inputs.absoluteRotationDegrees - encoderOffsetDegrees);
    }

    @Override
    public boolean isAngleOnTarget() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    protected ErrorCode setRotationSensorDegrees(double sensorDegrees) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getDriveDistanceInches() {
        return (inputs.leftMotorPosition + inputs.rightMotorPosition) / 2.0;
    }

    @Override
    public double getDriveVelocityInchesPerSecond() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getDriveVelocitySetpoint() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getDriveVoltage() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean isDrivePositionOnTarget() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void setDriveNeutralMode(NeutralMode mode) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setRotationNeutralMode(NeutralMode mode) {
        // TODO Auto-generated method stub

    }

    @Override
    public void setOpenLoop(Translation2d driveVector) {
        Rotation2d driveDirection = driveVector.direction();

        if (Util.shouldReverse(driveDirection, getAngle())) {
            outputs.rotationSetpoint = driveDirection.rotateBy(Rotation2d.fromDegrees(180.0));
            outputs.driveSetpoint = -driveVector.norm();
        } else {
            outputs.rotationSetpoint = driveDirection;
            outputs.driveSetpoint = driveVector.norm();
        }

        outputs.controlMode = ControlMode.PercentOutput;
    }

    @Override
    public void setOpenLoopCoast(Rotation2d driveDirection) {
        if (Util.shouldReverse(driveDirection, getAngle())) {
            outputs.rotationSetpoint = driveDirection.rotateBy(Rotation2d.fromDegrees(180.0));
        } else {
            outputs.rotationSetpoint = driveDirection;
        }

        outputs.controlMode = ControlMode.PercentOutput;
        outputs.driveSetpoint = 0.0;
    }

    @Override
    public void setClosedLoopVelocity(Translation2d driveVector) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setClosedLoopVelocityStall(Rotation2d driveDirection) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setClosedLoopPosition(Translation2d driveVector) {
        // TODO Auto-generated method stub
        
    }

    /**
     * If the desired drive speed and rotation speed cannot be achieved at the same time
     * and result in a violation of motor constraints (i.e., requesting one motor to go
     * faster than 100%), then we'll compromise by maintaining the same differential
     * (rotation speed) at the expense of lowering the drive speed.
     */
    private void boundMotorDemandsToConstraints() {
        double excessDemand = 0.0;

        if (outputs.leftMotorDemand > kMaxOpenLoopMotorOutput) {
            excessDemand = outputs.leftMotorDemand - kMaxOpenLoopMotorOutput;
        } else if (outputs.leftMotorDemand < -kMaxOpenLoopMotorOutput) {
            excessDemand = outputs.leftMotorDemand + kMaxOpenLoopMotorOutput;
        } else if (outputs.rightMotorDemand > kMaxOpenLoopMotorOutput) {
            excessDemand = outputs.rightMotorDemand - kMaxOpenLoopMotorOutput;
        } else if (outputs.leftMotorDemand < -kMaxOpenLoopMotorOutput) {
            excessDemand = outputs.rightMotorDemand + kMaxOpenLoopMotorOutput;
        }

        outputs.leftMotorDemand -= excessDemand;
        outputs.rightMotorDemand -= excessDemand;
    }

    private final Loop loop = new Loop() {
        double previousTimestamp = 0;

        @Override
        public void onStart(double timestamp) {
            previousTimestamp = timestamp;
        }

        @Override
        public void onLoop(double timestamp) {
            double errorDegrees = getAngle().minus(outputs.rotationSetpoint).getDegrees();
            double differentialOutput = openLoopRotationPID.calculate(errorDegrees, timestamp - previousTimestamp);
            differentialOutput = Util.limit(differentialOutput, kMaxOpenLoopDifferentialOutput);
            outputs.driveSetpoint = Util.limit(outputs.driveSetpoint, kMaxOpenLoopDriveSetpoint);
            outputs.leftMotorDemand = outputs.driveSetpoint - (differentialOutput / 2.0);
            outputs.rightMotorDemand = outputs.driveSetpoint + (differentialOutput / 2.0);
            boundMotorDemandsToConstraints();

            previousTimestamp = timestamp;
        }

        @Override
        public void onStop(double timestamp) {
        }
    };

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(loop);
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void disable() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void readPeriodicInputs() {
        inputs.leftMotorPosition = leftMotor.getSelectedSensorPosition();
        inputs.leftMotorVelocity = leftMotor.getVelocityEncoderUnitsPer100Ms();
        inputs.rightMotorPosition = rightMotor.getSelectedSensorPosition();
        inputs.rightMotorVelocity = rightMotor.getVelocityEncoderUnitsPer100Ms();
        inputs.absoluteRotationDegrees = rotationAbsoluteEncoder.getDegrees();
        Logger.getInstance().processInputs(name, inputs);
    }

    @Override
    public void writePeriodicOutputs() {
        leftMotor.set(outputs.controlMode, outputs.leftMotorDemand);
        rightMotor.set(outputs.controlMode, outputs.rightMotorDemand);
    }

    @Override
    public void outputTelemetry() {
        Logger.getInstance().recordOutput(String.format("%s/Angle", name), getAngle().getDegrees());
        Logger.getInstance().recordOutput(String.format("%s/Drive Distance", name), getDriveDistanceInches());
        Logger.getInstance().recordOutput(String.format("%s/Left Motor Demand", name), outputs.leftMotorDemand);
        Logger.getInstance().recordOutput(String.format("%s/Right Motor Demand", name), outputs.rightMotorDemand);
        Logger.getInstance().recordOutput(String.format("%s/Average Drive Output", name), (outputs.leftMotorDemand + outputs.rightMotorDemand) / 2.0);
        Logger.getInstance().recordOutput(String.format("%s/Drive Setpoint", name), outputs.driveSetpoint);
        Logger.getInstance().recordOutput(String.format("%s/Rotation Setpoint", name), outputs.rotationSetpoint.getDegrees());
        Logger.getInstance().recordOutput(String.format("%s/Left Current", name), leftMotor.getSupplyAmps());
        Logger.getInstance().recordOutput(String.format("%s/Right Current", name), rightMotor.getSupplyAmps());
    }

    @AutoLog
    public static class DifferentialSwerveInputs {
        public double leftMotorPosition;
        public double leftMotorVelocity;
        public double rightMotorPosition;
        public double rightMotorVelocity;
        public double absoluteRotationDegrees;
    }

    public static class DifferentialSwerveOutputs {
        public ControlMode controlMode = ControlMode.PercentOutput;
        public double leftMotorDemand = 0.0;
        public double rightMotorDemand = 0.0;
        public double driveSetpoint = 0.0;
        public Rotation2d rotationSetpoint = Rotation2d.identity();
    }
}
