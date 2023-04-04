package com.team1323.frc2023.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.subsystems.encoders.PhoenixProCANCoder;
import com.team1323.frc2023.subsystems.requests.Prerequisite;
import com.team1323.frc2023.subsystems.requests.Request;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemWithAbsoluteEncoder;
import com.team1323.lib.drivers.PhoenixProFXMotorController;
import com.team1323.lib.util.Netlink;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoulder extends ServoSubsystemWithAbsoluteEncoder<PhoenixProFXMotorController> {
    private static Shoulder instance = null;
    public static Shoulder getInstance() {
        if (instance == null) {
            instance = new Shoulder();
        }
        return instance;
    }
    
    public Shoulder() {
        super(new PhoenixProFXMotorController(Constants.Shoulder.kConfig.leaderPortNumber, Constants.Shoulder.kConfig.canBus),
                new ArrayList<>(), Constants.Shoulder.kConfig, Constants.Shoulder.kCurrentZeroingConfig,
                new PhoenixProCANCoder(Ports.SHOULDER_ENCODER, true), Constants.Shoulder.kAbsoluteEncoderInfo);
        leader.useCANCoder(Ports.SHOULDER_ENCODER);
        leader.config_IntegralZone(0, outputUnitsToEncoderUnits(2.0));
        leader.setPIDF(Constants.Shoulder.kPIDF);
        leader.setInverted(TalonFXInvertType.CounterClockwise);
        leader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.Shoulder.kContinuousSupplyCurrentLimit, 
                Constants.Shoulder.kTriggerSupplyCurrentLimit, 0.5));
        setPositionToAbsolute();
        stop();
    }

    @Override
    protected void setSensorPosition(double encoderUnits) {
        absoluteEncoder.setPosition(encoderUnits / 4096.0 * 360.0);
    }

    public void setAccelerationScalar(double scalar) {
        leader.configMotionAcceleration(config.maxEncoderVelocity * scalar);
    }

    /**
     * Assumes that the shoulder is at 0 degrees when completely horizontal (i.e., when gravity exerts the most
     * torque on the motor), and that rotating the shoulder upward results in a positive angle.
     */
    private void updateArbitraryFeedForward() {
        Rotation2d currentAngle = Rotation2d.fromDegrees(getPosition());
        periodicIO.arbitraryFeedForward = Constants.Shoulder.kArbitraryFeedForward * currentAngle.cos();
    }

    private Loop loop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            updateArbitraryFeedForward();
        }

        @Override
        public void onLoop(double timestamp) {
            updateArbitraryFeedForward();
        }

        @Override
        public void onStop(double timestamp) {}
    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        super.registerEnabledLoops(enabledLooper);
        enabledLooper.register(loop);
    }

    public Request angleRequest(double degrees) {
        return new Request() {
            @Override
            public void act() {
                setPosition(degrees);
            }

            @Override
            public boolean isFinished() {
                return isOnTarget();
            }

            @Override
            public String toString() {
                return String.format("ShoulderRequest(target = %.2f)", degrees);
            }
        };
    }

    public Prerequisite anglePrerequisite(double degrees) {
        return () -> isAtPosition(degrees);
    }
    private boolean neutralModeIsBrake = true;

    @Override
    public void outputTelemetry() {
        if(Netlink.getBooleanValue("Subsystems Coast Mode") && neutralModeIsBrake) {
			leader.setNeutralMode(NeutralMode.Coast);
			neutralModeIsBrake = false;
		} else if(!neutralModeIsBrake && !Netlink.getBooleanValue("Subsystems Coast Mode")) {
            leader.setNeutralMode(NeutralMode.Brake);
			neutralModeIsBrake = true;
		}
        SmartDashboard.putNumber("Shoulder Angle", getPosition());
        SmartDashboard.putNumber("Shoulder Target Angle", encoderUnitsToOutputUnits(periodicIO.demand));

        SmartDashboard.putNumber("Shoulder Encoder Position", periodicIO.position);
        SmartDashboard.putNumber("Shoulder Absolute Encoder", absoluteEncoder.getDegrees());
        SmartDashboard.putBoolean("Shoulder Is On Target", isOnTarget());
    }
}
