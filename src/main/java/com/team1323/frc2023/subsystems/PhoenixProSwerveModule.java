package com.team1323.frc2023.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.team254.drivers.LazyPhoenixProTalonFX;
import com.team254.lib.geometry.Translation2d;

public class PhoenixProSwerveModule extends SwerveModule {
    private final LazyPhoenixProTalonFX rotationMotor, driveMotor;

    private final TalonFXConfiguration rotationConfiguration = new TalonFXConfiguration();
    private final VoltageOut rotationVoltageOutRequest = new VoltageOut(0.0, true, false);
    private final MotionMagicVoltage rotationMotionMagicRequest = new MotionMagicVoltage(0.0, true, 0.0, 0, false);

    private final TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
    private final VoltageOut driveVoltageOutRequest = new VoltageOut(0.0, true, false);
    private final MotionMagicVoltage driveMotionMagicRequest = new MotionMagicVoltage(0.0, true, 0.0, 0, false);
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0.0, true, 0.0, 1, false);

    public PhoenixProSwerveModule(int rotationPort, int drivePort, int moduleId, 
            double encoderOffset, Translation2d startingPose, boolean flipAbsoluteEncoder) {
        super(moduleId, encoderOffset, startingPose, flipAbsoluteEncoder);
        rotationMotor = new LazyPhoenixProTalonFX(rotationPort);
        driveMotor = new LazyPhoenixProTalonFX(drivePort);
    }

    @Override
    protected void configureMotors() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void invertDriveMotor(TalonFXInvertType invertType) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void invertRotationMotor(TalonFXInvertType invertType) {
        // TODO Auto-generated method stub
        
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
    public boolean angleOnTarget() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    protected void setDriveProfileSlot(int slotIndex) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean drivePositionOnTarget() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    protected ErrorCode setRotationSensorPosition(double sensorPosition) {
        // TODO Auto-generated method stub
        return null;
    }
    
}
