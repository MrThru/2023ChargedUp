package com.team1323.frc2023.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2023.Constants;
import com.team1323.frc2023.Ports;
import com.team1323.frc2023.loops.ILooper;
import com.team1323.frc2023.loops.Loop;
import com.team1323.frc2023.requests.Request;
import com.team1323.frc2023.subsystems.encoders.Phoenix5CANCoder;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemWithAbsoluteEncoder;
import com.team1323.frc2023.subsystems.servo.ServoSubsystemWithAbsoluteEncoderInputs;
import com.team1323.lib.drivers.Phoenix5FXMotorController;
import com.team1323.lib.util.Netlink;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wrist extends ServoSubsystemWithAbsoluteEncoder<ServoSubsystemWithAbsoluteEncoderInputs> {
    private static Wrist instance = null;
    public static Wrist getInstance() {
        if (instance == null) {
            instance = new Wrist();
        }
        return instance;
    }

    private Wrist() {
        super(Phoenix5FXMotorController.createRealOrSimulatedController(Constants.Wrist.kConfig.leaderPortNumber, Constants.Wrist.kConfig.canBus), 
                new ArrayList<>(), Constants.Wrist.kConfig, Constants.Wrist.kCurrentZeroingConfig,
                Phoenix5CANCoder.createRealOrSimulatedEncoder(Ports.WRIST_ENCODER, false), 
                Constants.Wrist.kAbsoluteEncoderInfo, new ServoSubsystemWithAbsoluteEncoderInputs());
        leader.useIntegratedSensor();
        leader.config_IntegralZone(0, outputUnitsToEncoderUnits(4.0));
        leader.setPIDF(Constants.Wrist.kPIDF);
        leader.setInverted(TalonFXInvertType.CounterClockwise);
        setSupplyCurrentLimit(Constants.Wrist.kSupplyCurrentLimit);
        setStatorCurrentLimit(200);
        setPositionToAbsolute();
        stop();
    }

    /**
     * Assumes that the wrist is at 0 degrees when completely horizontal (i.e., when gravity exerts the most
     * torque on the motor), and that rotating the wrist upward results in a positive angle.
     */
    private void updateArbitraryFeedForward() {
        Rotation2d currentAngle = Rotation2d.fromDegrees(getPosition());
        outputs.arbitraryFeedForward = Constants.Wrist.kArbitraryFeedForward * currentAngle.cos();
    }

    
    private double targetAmpsOnTarget = 0;
    public void setCurrentAtTarget(double amps) {
        targetAmpsOnTarget = amps;
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
                return String.format("WristRequest(target = %.2f)", degrees);
            }
        };
    }

    public Request setStatorCurrentLimitRequest(double amps) {
        return new Request() {
            @Override
            public void act() {
                setStatorCurrentLimit(amps);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    public Request setCurrentAtTargetRequest(double amps) {
        return new Request() {
            @Override
            public void act() {
                setCurrentAtTarget(amps);
            }
        };
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
        SmartDashboard.putNumber("Wrist Angle", getPosition());
        SmartDashboard.putNumber("Wrist Absolute Encoder", absoluteEncoder.getDegrees());
        SmartDashboard.putBoolean("Wrist On Target", isOnTarget());
        SmartDashboard.putNumber("Wrist Target Angle", encoderUnitsToOutputUnits(outputs.demand));
    }
}
