package com.team1323.frc2023.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.team254.lib.geometry.Translation2d;

public class PhoenixProSwerveModule extends SwerveModule {
    public PhoenixProSwerveModule(int rotationPort, int drivePort, int moduleId,
            double encoderOffset, Translation2d startingPose, boolean flipAbsoluteEncoder) {
        super(rotationPort, drivePort, moduleId, encoderOffset, startingPose, flipAbsoluteEncoder);
    }

    @Override
    protected void configureMotors() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();
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
